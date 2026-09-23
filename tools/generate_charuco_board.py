#!/usr/bin/env python3
"""Generate the fixed-size ChArUco target used by the desk-cleaning calibration.

The board geometry is deliberately kept in millimetres. The PNG is a 300 DPI
rendering; the PDF places the same image at its physical size on an A4 page.
"""
import argparse
from pathlib import Path

import cv2
from PIL import Image


SQUARES_X = 7
SQUARES_Y = 5
SQUARE_LENGTH_MM = 25
MARKER_LENGTH_MM = 18
DPI = 300


def make_board():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'CharucoBoard_create'):
        return cv2.aruco.CharucoBoard_create(
            SQUARES_X, SQUARES_Y,
            SQUARE_LENGTH_MM / 1000.0,
            MARKER_LENGTH_MM / 1000.0,
            dictionary,
        )
    return cv2.aruco.CharucoBoard.create(
        SQUARES_X, SQUARES_Y,
        SQUARE_LENGTH_MM / 1000.0,
        MARKER_LENGTH_MM / 1000.0,
        dictionary,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output-dir', default='sources/calibration',
        help='directory for generated board files',
    )
    args = parser.parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    board = make_board()
    width_px = round(SQUARES_X * SQUARE_LENGTH_MM / 25.4 * DPI)
    height_px = round(SQUARES_Y * SQUARE_LENGTH_MM / 25.4 * DPI)
    image = board.draw((width_px, height_px), marginSize=0, borderBits=1)
    png_path = output_dir / 'charuco_7x5_25mm_18mm_dict4x4_50_300dpi.png'
    Image.fromarray(image).save(str(png_path), dpi=(DPI, DPI))

    from reportlab.lib.pagesizes import A4
    from reportlab.lib.units import mm
    from reportlab.pdfgen import canvas

    pdf_path = output_dir / 'charuco_7x5_25mm_18mm_dict4x4_50.pdf'
    page_width, page_height = A4
    board_width = SQUARES_X * SQUARE_LENGTH_MM * mm
    board_height = SQUARES_Y * SQUARE_LENGTH_MM * mm
    x = (page_width - board_width) / 2.0
    y = page_height - 35 * mm - board_height

    pdf = canvas.Canvas(str(pdf_path), pagesize=A4)
    pdf.setTitle('ArmPi FPV eye-in-hand ChArUco calibration board')
    pdf.setFont('Helvetica-Bold', 11)
    pdf.drawCentredString(
        page_width / 2.0, page_height - 20 * mm,
        'ArmPi FPV ChArUco calibration board',
    )
    pdf.drawImage(
        str(png_path), x, y, width=board_width, height=board_height,
        preserveAspectRatio=False, mask='auto',
    )

    # A physical-size check that can be measured after printing.
    bar_x = x
    bar_y = y - 16 * mm
    pdf.setLineWidth(1)
    pdf.line(bar_x, bar_y, bar_x + 100 * mm, bar_y)
    pdf.line(bar_x, bar_y - 2 * mm, bar_x, bar_y + 2 * mm)
    pdf.line(bar_x + 100 * mm, bar_y - 2 * mm, bar_x + 100 * mm, bar_y + 2 * mm)
    pdf.setFont('Helvetica', 8)
    pdf.drawString(bar_x, bar_y - 6 * mm, '100 mm check: measure after printing')
    pdf.drawRightString(
        page_width - 15 * mm, 15 * mm,
        '7x5 squares | square 25 mm | marker 18 mm | DICT_4X4_50 | print at 100%',
    )
    pdf.showPage()
    pdf.save()
    print('wrote %s' % png_path)
    print('wrote %s' % pdf_path)


if __name__ == '__main__':
    main()
