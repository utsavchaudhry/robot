const COOKIE_NAME = "rec_auth";
const COOKIE_MAX_AGE = 60 * 60 * 24 * 30; // 30 days

export default {
  async fetch(request, env) {
    const url = new URL(request.url);

    if (!env.PASSWORD) {
      return new Response(
        "Server misconfigured: PASSWORD secret not set. Run `wrangler secret put PASSWORD`.",
        { status: 500, headers: { "content-type": "text/plain" } }
      );
    }

    const expectedToken = await sha256Hex(env.PASSWORD);

    if (url.pathname === "/__login") {
      if (request.method === "POST") {
        const form = await request.formData();
        const submitted = (form.get("password") || "").toString();
        if (timingSafeEqual(await sha256Hex(submitted), expectedToken)) {
          const next = sanitizeNext(url.searchParams.get("next"));
          return new Response(null, {
            status: 303,
            headers: {
              "set-cookie": `${COOKIE_NAME}=${expectedToken}; Path=/; HttpOnly; Secure; SameSite=Lax; Max-Age=${COOKIE_MAX_AGE}`,
              "location": next,
            },
          });
        }
        return loginPage(url, { error: "Wrong password.", status: 401 });
      }
      return loginPage(url);
    }

    if (url.pathname === "/__logout") {
      return new Response(null, {
        status: 303,
        headers: {
          "set-cookie": `${COOKIE_NAME}=; Path=/; HttpOnly; Secure; SameSite=Lax; Max-Age=0`,
          "location": "/__login",
        },
      });
    }

    const cookies = parseCookies(request.headers.get("cookie") || "");
    const presented = cookies[COOKIE_NAME];
    if (!presented || !timingSafeEqual(presented, expectedToken)) {
      const next = encodeURIComponent(url.pathname + url.search);
      return Response.redirect(`${url.origin}/__login?next=${next}`, 303);
    }

    let key = url.pathname.slice(1);

    if (key && !key.endsWith("/")) {
      const object = await env.BUCKET.get(key);
      if (object) {
        return new Response(object.body, {
          headers: {
            "content-type": getMime(key),
            "content-length": object.size,
            "cache-control": "private, max-age=86400",
          },
        });
      }
    }

    if (key && !key.endsWith("/")) key += "/";
    if (key === "/") key = "";

    const list = await env.BUCKET.list({ prefix: key, delimiter: "/" });

    const folders = (list.delimitedPrefixes || []).map((p) => {
      const name = p.replace(key, "").replace(/\/$/, "");
      return `<li><a href="/${p}">${name}/</a></li>`;
    });

    const files = list.objects.map((obj) => {
      const name = obj.key.replace(key, "");
      if (!name) return "";
      const size = formatSize(obj.size);
      const date = new Date(obj.uploaded).toLocaleString();
      return `<li><a href="/${obj.key}">${name}</a> <span>(${size}, ${date})</span></li>`;
    });

    const breadcrumb = buildBreadcrumb(key);
    const html = `<!DOCTYPE html>
<html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Teleop Recordings${key ? " — " + key : ""}</title>
<style>
  body { font-family: system-ui, sans-serif; max-width: 800px; margin: 2rem auto; padding: 0 1rem; color: #222; }
  h1 { font-size: 1.3rem; display: flex; justify-content: space-between; align-items: baseline; }
  h1 a.logout { font-size: 0.7rem; font-weight: 400; color: #888; text-decoration: none; }
  h1 a.logout:hover { color: #c00; }
  .breadcrumb { font-size: 0.9rem; color: #666; margin-bottom: 1rem; }
  .breadcrumb a { color: #0066cc; }
  ul { list-style: none; padding: 0; }
  li { padding: 0.4rem 0; border-bottom: 1px solid #eee; display: flex; justify-content: space-between; align-items: center; }
  li a { color: #0066cc; text-decoration: none; font-weight: 500; }
  li a:hover { text-decoration: underline; }
  li span { color: #888; font-size: 0.85rem; }
  .empty { color: #999; font-style: italic; }
</style>
</head><body>
<h1>Teleop Recordings <a class="logout" href="/__logout">log out</a></h1>
<div class="breadcrumb">${breadcrumb}</div>
<ul>
${folders.join("\n")}
${files.join("\n")}
</ul>
${folders.length + files.length === 0 ? '<p class="empty">No recordings yet.</p>' : ""}
</body></html>`;

    return new Response(html, {
      headers: {
        "content-type": "text/html; charset=utf-8",
        "cache-control": "private, no-store",
      },
    });
  },
};

function loginPage(url, { error = null, status = 200 } = {}) {
  const next = sanitizeNext(url.searchParams.get("next"));
  const action = `/__login?next=${encodeURIComponent(next)}`;
  const html = `<!DOCTYPE html>
<html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Teleop Recordings — Sign in</title>
<style>
  body { font-family: system-ui, sans-serif; max-width: 360px; margin: 15vh auto; padding: 0 1rem; color: #222; }
  h1 { font-size: 1.1rem; margin-bottom: 1rem; }
  form { display: flex; flex-direction: column; gap: 0.6rem; }
  input[type=password] { font-size: 1rem; padding: 0.5rem 0.6rem; border: 1px solid #ccc; border-radius: 6px; }
  button { font-size: 0.95rem; padding: 0.5rem 0.8rem; border: 0; border-radius: 6px; background: #0066cc; color: white; cursor: pointer; }
  button:hover { background: #0055a8; }
  .error { color: #c00; font-size: 0.85rem; margin-top: 0.4rem; }
</style>
</head><body>
<h1>Teleop Recordings</h1>
<form method="post" action="${action}">
  <input type="password" name="password" autofocus autocomplete="current-password" placeholder="Password" required>
  <button type="submit">Sign in</button>
  ${error ? `<div class="error">${error}</div>` : ""}
</form>
</body></html>`;
  return new Response(html, {
    status,
    headers: {
      "content-type": "text/html; charset=utf-8",
      "cache-control": "no-store",
    },
  });
}

function sanitizeNext(next) {
  if (!next || !next.startsWith("/") || next.startsWith("//")) return "/";
  return next;
}

function parseCookies(header) {
  const out = {};
  header.split(/;\s*/).forEach((c) => {
    if (!c) return;
    const idx = c.indexOf("=");
    if (idx > 0) out[c.slice(0, idx)] = c.slice(idx + 1);
  });
  return out;
}

async function sha256Hex(str) {
  const buf = await crypto.subtle.digest("SHA-256", new TextEncoder().encode(str));
  return Array.from(new Uint8Array(buf))
    .map((b) => b.toString(16).padStart(2, "0"))
    .join("");
}

function timingSafeEqual(a, b) {
  if (typeof a !== "string" || typeof b !== "string" || a.length !== b.length) return false;
  let result = 0;
  for (let i = 0; i < a.length; i++) result |= a.charCodeAt(i) ^ b.charCodeAt(i);
  return result === 0;
}

function getMime(key) {
  if (key.endsWith(".mp4")) return "video/mp4";
  if (key.endsWith(".avi")) return "video/x-msvideo";
  if (key.endsWith(".wav")) return "audio/wav";
  if (key.endsWith(".json")) return "application/json";
  return "application/octet-stream";
}

function formatSize(bytes) {
  if (bytes < 1024) return bytes + " B";
  if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(1) + " KB";
  if (bytes < 1024 * 1024 * 1024) return (bytes / (1024 * 1024)).toFixed(1) + " MB";
  return (bytes / (1024 * 1024 * 1024)).toFixed(1) + " GB";
}

function buildBreadcrumb(key) {
  const parts = key.split("/").filter(Boolean);
  let path = "";
  let crumbs = [`<a href="/">home</a>`];
  for (const part of parts) {
    path += part + "/";
    crumbs.push(`<a href="/${path}">${part}</a>`);
  }
  return crumbs.join(" / ");
}
