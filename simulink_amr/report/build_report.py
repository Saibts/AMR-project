# -*- coding: utf-8 -*-
"""Builds the project report in the format of the reference report (Tracer_MATLAB_Report):
PDF (HTML -> Edge headless, cover/frame/page numbers drawn with PyMuPDF) and DOCX (python-docx)."""
import os, re, glob, subprocess, html
import content as C

HERE = C.HERE
OUT_PDF = os.path.join(HERE, "Scout_Mini_Warehouse_AMR_Report.pdf")
OUT_DOCX = os.path.join(HERE, "Scout_Mini_Warehouse_AMR_Report.docx")
EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"
LOGO = os.path.join(C.FIG, "au_logo.png")
WF = r"C:\Windows\Fonts"

COURSE = "RA23502 \u2013 AUTONOMOUS MOBILE ROBOT"
TITLE_L1, TITLE_L2 = "DESIGN AND SIMULATION OF INDUSTRY", "STANDARD AMRs (AGILEX SCOUT MINI)"
TITLE_CAP = "Design and Simulation of Industry Standard AMRs (AgileX Scout Mini)"
NAME_UP = "SAILAKSHMI B T (2024511019)"
NAME = "Sailakshmi B T (2024511019)"
ORANGE = (0xBF / 255, 0x4E / 255, 0x14 / 255)

B = list(C.B)
heads = [(b[0], b[1]) for b in B if b[0] in ("h1", "h2")]

MONO = re.compile(r"\b([A-Za-z_][\w]*\.(?:m|slx|urdf|STL|SLDPRT|SLDASM|py|mat|json)|importrobot|smimport|rigidBodyTree|tc_on|ode23t|package://[\w/]+|meshes/|run_scout_warehouse_simulation(?:\([^)]*\))?)")


def inline_html(t):
    t = html.escape(t)
    return MONO.sub(lambda m: f"<code>{m.group(1)}</code>", t)


def img_uri(path): return "file:///" + path.replace("\\", "/")


def render_html(toc_pages):
    css = """
    @page { size: A4; margin: 72pt 72pt 72pt 72pt; }
    body { font-family: 'Times New Roman', Times, serif; font-size: 12pt; line-height: 16pt; color: #000; text-align: justify; margin: 0; }
    h1 { font-size: 12pt; font-weight: bold; text-align: center; margin: 14pt 0 8pt 0; line-height: 16pt; page-break-after: avoid; }
    h2 { font-size: 12pt; font-weight: bold; text-align: left; margin: 10pt 0 6pt 0; line-height: 16pt; page-break-after: avoid; }
    p { margin: 0 0 8pt 0; }
    ul { margin: 0 0 8pt 0; padding-left: 36pt; }
    li { margin-bottom: 8pt; padding-left: 0; }
    code { font-family: 'Courier New', monospace; font-size: 11pt; }
    pre { font-family: 'Courier New', monospace; font-size: 10pt; line-height: 12pt; white-space: pre-wrap; text-align: left; margin: 0 0 8pt 0; page-break-inside: avoid; }
    figure { margin: 4pt 0 10pt 0; text-align: center; page-break-inside: avoid; }
    figure img { max-width: 100%; }
    figcaption { font-size: 11pt; line-height: 14pt; text-align: center; margin-top: 4pt; }
    table { width: 100%; border-collapse: collapse; margin: 0 0 8pt 0; font-size: 9.5pt; line-height: 11.5pt; text-align: left; }
    th, td { border: 0.75pt solid #000; padding: 1.5pt 4pt; vertical-align: middle; }
    th { text-align: center; font-weight: bold; }
    tr { page-break-inside: avoid; }
    td code { font-size: 9.5pt; }
    .cover { page-break-after: always; height: 10pt; }
    .cert p { font-size: 14pt; line-height: 16pt; text-align: left; margin: 0 0 16pt 0; }
    .toc { width: 415pt; font-size: 11pt; }
    .toc th, .toc td { font-weight: bold; text-align: center; padding: 3pt 4pt; font-size: 11pt; line-height: 14pt; }
    .toc td.tp { text-align: left; }
    """
    o = [f"<!doctype html><html><head><meta charset='utf-8'><title>{html.escape(TITLE_CAP)}</title><style>{css}</style></head><body>"]
    o.append("<div class='cover'></div>")
    # certificate
    o.append("<div class='cert' style='page-break-after:always'>"
             "<h1 style='font-size:18pt;line-height:20pt;margin:0 0 24pt 0'>BONAFIDE CERTIFICATE</h1>"
             f"<p>Certified that this project report &ldquo;Design and Simulation of Industry Standard AMRs (AgileX Scout Mini)&rdquo; is the Bonafide work of</p>"
             f"<p>{html.escape(NAME)} who carried out the</p><p>project work under my supervision.</p>"
             "<div style='height:150pt'></div>"
             "<table style='border:0;width:100%;font-size:14pt;line-height:16pt'><tr>"
             "<td style='border:0;text-align:left;padding:0 0 28pt 0'><b>SIGNATURE</b></td><td style='border:0;text-align:right;padding:0 0 28pt 0;width:50%'><b>SIGNATURE</b></td></tr><tr>"
             "<td style='border:0;text-align:left;padding:0'><b>HEAD OF THE DEPARTMENT</b></td><td style='border:0;text-align:right;padding:0;'><b>SUPERVISOR</b></td></tr></table></div>")
    # TOC
    o.append("<div style='page-break-after:always'><h1 style='margin:24pt 0 24pt 0'>TABLE OF CONTENTS:</h1><table class='toc'>"
             "<colgroup><col style='width:50pt'><col style='width:320pt'><col style='width:45pt'></colgroup>"
             "<tr><th>S.No</th><th>TOPICS</th><th>PAGE</th></tr>")
    for k, t in [h for h in heads if h[0] == "h1"]:
        num, _, title = t.partition(". ")
        o.append(f"<tr><td>{num}</td><td class='tp'>{html.escape(title)}</td><td>{toc_pages.get(t, '')}</td></tr>")
    o.append("</table></div>")
    o.append(f"<div style='page-break-after:always'><h1 style='margin:0 0 8pt 0'>Abstract</h1><p>{inline_html(C.ABSTRACT)}</p></div>")
    for b in B:
        k = b[0]
        if k == "h1":
            o.append(f"<h1>{html.escape(b[1])}</h1>")
        elif k == "h2":
            o.append(f"<h2>{html.escape(b[1])}</h2>")
        elif k == "p":
            o.append(f"<p>{inline_html(b[1])}</p>")
        elif k == "ul":
            o.append("<ul>" + "".join(f"<li>{inline_html(x)}</li>" for x in b[1]) + "</ul>")
        elif k == "code":
            o.append(f"<pre>{html.escape(b[1])}</pre>")
        elif k == "pagebreak":
            o.append("<div style='page-break-after:always;height:1pt'></div>")
        elif k == "fig":
            o.append(f"<figure><img src='{img_uri(b[1])}' style='width:{int(b[3]*100)}%'><figcaption>{html.escape(b[2])}</figcaption></figure>")
        elif k == "table":
            _, cap, head, rows, widths = b
            o.append("<table>")
            if widths:
                o.append("<colgroup>" + "".join(f"<col style='width:{int(w*100)}%'>" for w in widths) + "</colgroup>")
            o.append("<tr>" + "".join(f"<th>{html.escape(h)}</th>" for h in head) + "</tr>")
            for r in rows:
                o.append("<tr>" + "".join(f"<td>{inline_html(c)}</td>" for c in r) + "</tr>")
            o.append("</table>")
    o.append("</body></html>")
    return "\n".join(o)


def make_pdf(toc_pages, tag):
    htmlp = os.path.join(HERE, f"report_{tag}.html")
    open(htmlp, "w", encoding="utf-8").write(render_html(toc_pages))
    pdf = os.path.join(HERE, f"report_{tag}.pdf")
    if os.path.exists(pdf): os.remove(pdf)
    subprocess.run([EDGE, "--headless=new", "--disable-gpu", "--no-pdf-header-footer", "--allow-file-access-from-files",
                    f"--print-to-pdf={pdf}", img_uri(htmlp)], check=True, timeout=300)
    return pdf


def find_pages(pdf):
    import fitz
    d = fitz.open(pdf)
    texts = [p.get_text() for p in d]
    pages = {}
    for k, t in heads:
        if k != "h1": continue
        for i, tx in enumerate(texts):
            if i < 4: continue
            if re.search(r"^\s*" + re.escape(t) + r"\s*$", tx, re.M):
                pages[t] = i + 1; break
    return pages, len(d)


def finish_pdf(src, dst):
    """Cover page, page frame and page numbers, as in the reference report."""
    import fitz
    d = fitz.open(src)
    tnr, tnrb, tnri = (fitz.Font(fontfile=os.path.join(WF, f)) for f in ("times.ttf", "timesbd.ttf", "timesi.ttf"))
    for i, p in enumerate(d):
        W, H = p.rect.width, p.rect.height
        p.draw_rect(fitz.Rect(24, 24, W - 24, H - 24), color=(0, 0, 0), width=0.5)
        tw = fitz.TextWriter(p.rect)
        if i == 0:
            def center(txt, y, font, size, color=(0, 0, 0)):
                w = font.text_length(txt, fontsize=size)
                t2 = fitz.TextWriter(p.rect, color=color); t2.append((W / 2 - w / 2, y), txt, font=font, fontsize=size); t2.write_text(p)
            center(COURSE, 89, tnrb, 17.5)
            p.draw_rect(fitz.Rect(59.9, 99.0, 539.15, 173.4), color=(0, 0, 0), width=0.5, radius=0.18)
            center(TITLE_L1, 128, tnrb, 18, ORANGE); center(TITLE_L2, 149, tnrb, 18, ORANGE)
            center("Submitted By:", 232, tnrb, 16); center(NAME_UP, 268, tnrb, 16)
            center("BACHELOR OF ENGINEERING", 347, tnrb, 18); center("in", 368, tnri, 18); center("ROBOTICS AND AUTOMATION", 389, tnrb, 18)
            p.insert_image(fitz.Rect(204.65, 432.95, 390.65, 618.95), filename=LOGO)
            center("DEPARTMENT OF PRODUCTION TECHNOLOGY", 694, tnrb, 18)
            center("MADRAS INSTITUTE OF TECHNOLOGY", 715, tnrb, 18)
            center("ANNA UNIVERSITY - CHENNAI 600044", 736, tnrb, 18)
        # page number (Aptos 12 in the reference; Helvetica-like here)
        s = str(i + 1)
        p.insert_text((W / 2 - 3.2 * len(s), 789), s, fontsize=12, fontname="helv")
    d.save(dst)


def build_pdf():
    p1 = make_pdf({}, "pass1"); pages, n = find_pages(p1)
    p2 = make_pdf(pages, "pass2"); pages2, n2 = find_pages(p2)
    if pages2 != pages:
        p2 = make_pdf(pages2, "pass3"); pages = pages2
    finish_pdf(p2, OUT_PDF)
    print("PDF pages:", n2); print(pages)
    return pages


# ====================================================================================================== DOCX
def build_docx(toc_pages):
    from docx import Document
    from docx.shared import Pt, Cm, RGBColor
    from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK, WD_LINE_SPACING
    from docx.enum.table import WD_TABLE_ALIGNMENT, WD_CELL_VERTICAL_ALIGNMENT
    from docx.oxml.ns import qn
    from docx.oxml import OxmlElement
    doc = Document()
    sec = doc.sections[0]
    sec.page_width, sec.page_height = Cm(21), Cm(29.7)
    for a in ("left_margin", "right_margin", "top_margin", "bottom_margin"): setattr(sec, a, Cm(2.54))
    st = doc.styles["Normal"]; st.font.name = "Times New Roman"; st.font.size = Pt(12)
    st.element.rPr.rFonts.set(qn("w:eastAsia"), "Times New Roman")
    st.paragraph_format.line_spacing_rule = WD_LINE_SPACING.EXACTLY; st.paragraph_format.line_spacing = Pt(16)
    st.paragraph_format.space_after = Pt(8)
    for nm, al in (("Heading 1", WD_ALIGN_PARAGRAPH.CENTER), ("Heading 2", WD_ALIGN_PARAGRAPH.LEFT)):
        h = doc.styles[nm]; h.font.name = "Times New Roman"; h.font.size = Pt(12); h.font.bold = True
        h.font.color.rgb = RGBColor(0, 0, 0); h.element.rPr.rFonts.set(qn("w:eastAsia"), "Times New Roman")
        h.paragraph_format.alignment = al; h.paragraph_format.space_before = Pt(10); h.paragraph_format.space_after = Pt(6)
        h.paragraph_format.line_spacing_rule = WD_LINE_SPACING.EXACTLY; h.paragraph_format.line_spacing = Pt(16)
    pgb = OxmlElement("w:pgBorders"); pgb.set(qn("w:offsetFrom"), "page")
    for side in ("top", "left", "bottom", "right"):
        e = OxmlElement(f"w:{side}"); e.set(qn("w:val"), "single"); e.set(qn("w:sz"), "4"); e.set(qn("w:space"), "24"); e.set(qn("w:color"), "000000"); pgb.append(e)
    sec._sectPr.append(pgb)
    fp = sec.footer.paragraphs[0]; fp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = fp.add_run(); r.font.name = "Aptos"; r.font.size = Pt(12)
    for t, txt in (("begin", None), (None, "PAGE"), ("end", None)):
        if t:
            fc = OxmlElement("w:fldChar"); fc.set(qn("w:fldCharType"), t); r._r.append(fc)
        else:
            it = OxmlElement("w:instrText"); it.set(qn("xml:space"), "preserve"); it.text = txt; r._r.append(it)

    def para(text="", align=WD_ALIGN_PARAGRAPH.JUSTIFY, bold=False, size=None, space=8, italic=False, color=None, line=None):
        p = doc.add_paragraph(); p.alignment = align
        if text:
            parts = re.split(MONO, text) if align == WD_ALIGN_PARAGRAPH.JUSTIFY else [text]
            for j, part in enumerate(parts):
                if not part: continue
                r = p.add_run(part); r.bold = bold; r.italic = italic
                if size: r.font.size = Pt(size)
                if color: r.font.color.rgb = RGBColor(*color)
                if align == WD_ALIGN_PARAGRAPH.JUSTIFY and j % 2 == 1:
                    r.font.name = "Courier New"; r.font.size = Pt(11); r._element.rPr.rFonts.set(qn("w:eastAsia"), "Courier New")
        p.paragraph_format.space_after = Pt(space)
        if line:
            p.paragraph_format.line_spacing_rule = WD_LINE_SPACING.EXACTLY; p.paragraph_format.line_spacing = Pt(line)
        return p

    def brk(): doc.add_paragraph().add_run().add_break(WD_BREAK.PAGE)

    # ---- cover ----
    C_ = WD_ALIGN_PARAGRAPH.CENTER
    para(COURSE, C_, True, 17.5, 8)
    tp = para(TITLE_L1 + "\n" + TITLE_L2, C_, True, 18, 24, color=(0xBF, 0x4E, 0x14), line=20)
    pPr = tp._p.get_or_add_pPr(); bd = OxmlElement("w:pBdr")
    for side in ("top", "left", "bottom", "right"):
        e = OxmlElement(f"w:{side}"); e.set(qn("w:val"), "single"); e.set(qn("w:sz"), "4"); e.set(qn("w:space"), "6"); e.set(qn("w:color"), "000000"); bd.append(e)
    pPr.append(bd)
    para("", C_, space=14)
    para("Submitted By:", C_, True, 16, 14); para(NAME_UP, C_, True, 16, 30)
    para("BACHELOR OF ENGINEERING", C_, True, 18, 0, line=21); para("in", C_, False, 18, 0, italic=True, line=21)
    para("ROBOTICS AND AUTOMATION", C_, True, 18, 16, line=21)
    lp = doc.add_paragraph(); lp.alignment = C_; lp.add_run().add_picture(LOGO, width=Cm(6.56)); lp.paragraph_format.space_after = Pt(18)
    for d_ in ("DEPARTMENT OF PRODUCTION TECHNOLOGY", "MADRAS INSTITUTE OF TECHNOLOGY", "ANNA UNIVERSITY - CHENNAI 600044"):
        para(d_, C_, True, 18, 0, line=21)
    brk()
    # ---- certificate ----
    para("BONAFIDE CERTIFICATE", C_, True, 18, 22)
    L = WD_ALIGN_PARAGRAPH.LEFT
    para(f"Certified that this project report \u201c{TITLE_CAP}\u201d is the Bonafide work of", L, False, 14, 16)
    para(f"{NAME} who carried out the", L, False, 14, 16)
    para("project work under my supervision.", L, False, 14, 150)
    para("SIGNATURE\t\t\t\t\tSIGNATURE", L, True, 14, 28)
    para("HEAD OF THE DEPARTMENT\t\t\tSUPERVISOR", L, True, 14, 8)
    brk()
    # ---- TOC ----
    para("TABLE OF CONTENTS:", C_, True, 12, 18)
    tb = doc.add_table(rows=1, cols=3); tb.style = "Table Grid"
    for c, t in zip(tb.rows[0].cells, ("S.No", "TOPICS", "PAGE")):
        c.text = ""; rr = c.paragraphs[0].add_run(t); rr.bold = True; rr.font.size = Pt(11); c.paragraphs[0].alignment = C_
    for k, t in [h for h in heads if h[0] == "h1"]:
        num, _, title = t.partition(". ")
        cells = tb.add_row().cells
        for j, (c, v) in enumerate(zip(cells, (num, title, str(toc_pages.get(t, ""))))):
            c.text = ""; rr = c.paragraphs[0].add_run(v); rr.bold = True; rr.font.size = Pt(11)
            c.paragraphs[0].alignment = C_ if j != 1 else L
    for row in tb.rows:
        for c, w in zip(row.cells, (1.76, 11.3, 1.6)):
            c.width = Cm(w); c.paragraphs[0].paragraph_format.space_after = Pt(2); c.paragraphs[0].paragraph_format.line_spacing = Pt(14)
    brk()
    doc.add_heading("Abstract", 1); para(C.ABSTRACT)
    brk()
    for b in B:
        k = b[0]
        if k == "h1":
            doc.add_heading(b[1], 1)
        elif k == "h2":
            doc.add_heading(b[1], 2)
        elif k == "p":
            para(b[1])
        elif k == "ul":
            for x in b[1]:
                p = doc.add_paragraph(style="List Bullet"); p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
                for j, part in enumerate(re.split(MONO, x)):
                    if not part: continue
                    rr = p.add_run(part)
                    if j % 2 == 1: rr.font.name = "Courier New"; rr.font.size = Pt(11)
        elif k == "code":
            p = doc.add_paragraph(); p.paragraph_format.line_spacing_rule = WD_LINE_SPACING.EXACTLY; p.paragraph_format.line_spacing = Pt(12)
            r = p.add_run(b[1]); r.font.name = "Courier New"; r.font.size = Pt(10)
            r._element.rPr.rFonts.set(qn("w:eastAsia"), "Courier New")
        elif k == "pagebreak":
            doc.add_page_break()
        elif k == "fig":
            width = min(16.0, 16.0 * b[3])
            p = doc.add_paragraph(); p.alignment = C_
            p.add_run().add_picture(b[1], width=Cm(width)); p.paragraph_format.keep_with_next = True
            p.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
            para(b[2], C_, False, 11, 10, line=14)
        elif k == "table":
            _, cap, head, rows, widths = b
            t = doc.add_table(rows=1, cols=len(head)); t.style = "Table Grid"; t.alignment = WD_TABLE_ALIGNMENT.CENTER
            for c, h in zip(t.rows[0].cells, head):
                c.text = ""; r = c.paragraphs[0].add_run(h); r.bold = True; r.font.size = Pt(9.5); c.paragraphs[0].alignment = C_
            for row in rows:
                cells = t.add_row().cells
                for c, v in zip(cells, row):
                    c.text = ""; r = c.paragraphs[0].add_run(v); r.font.size = Pt(9.5)
            for row in t.rows:
                for j, c in enumerate(row.cells):
                    pf = c.paragraphs[0].paragraph_format; pf.space_after = Pt(1); pf.line_spacing_rule = WD_LINE_SPACING.EXACTLY; pf.line_spacing = Pt(11.5)
                    c.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
                    if widths: c.width = Cm(15.9 * widths[j])
            para("", L, space=4)
    doc.save(OUT_DOCX)
    print("DOCX saved:", OUT_DOCX)


if __name__ == "__main__":
    pages = build_pdf()
    build_docx(pages)
