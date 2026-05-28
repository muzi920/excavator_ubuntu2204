from pypdf import PdfReader
reader = PdfReader('e:/temp/shandong/数字量输入输出系列使用手册(CAN版).pdf')
with open('extracted_pdf.txt', 'w', encoding='utf-8') as f:
    for page in reader.pages:
        f.write(page.extract_text() + '\n')
