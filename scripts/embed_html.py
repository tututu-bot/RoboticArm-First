# 把 src/index.html 转成 src/index_html.c（编译期自动生成，无需手改）
# 只有 HTML 内容变化时才重写，避免每次全量重编
Import("env")
import os

src = os.path.join(env["PROJECT_DIR"], "src", "index.html")
dst = os.path.join(env["PROJECT_DIR"], "src", "index_html.c")

data = open(src, "rb").read()
try:
    old = open(dst, "r").read()
except OSError:
    old = ""

new = ("// 由 scripts/embed_html.py 自动生成，请勿手改（改 src/index.html 后重新编译即可）\n"
       "const unsigned char index_html[] = {\n"
       + ",\n".join("0x%02x" % b for b in data) + "\n};\n"
       "const unsigned int index_html_len = %d;\n" % len(data))

if new != old:
    open(dst, "w").write(new)
    print("embed_html: src/index_html.c 已更新（%d 字节）" % len(data))
