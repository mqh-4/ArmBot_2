#!/usr/bin/env python3
"""Create an actual-size, lightweight eye-to-hand ChArUco target on A4."""

from io import BytesIO
from pathlib import Path

import cv2
from PIL import Image
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader


SQUARES_X = 6
SQUARES_Y = 4
SQUARE_LENGTH_MM = 20
MARKER_LENGTH_MM = 15
DICTIONARY = cv2.aruco.DICT_4X4_50
RENDER_DPI = 600

OUTPUT = (
    Path(__file__).resolve().parents[1]
    / "output/pdf/armpi_fpv_eye_to_hand_charuco_6x4_20mm_15mm_a4.pdf"
)


def make_board():
    dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARY)
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        return cv2.aruco.CharucoBoard_create(
            SQUARES_X,
            SQUARES_Y,
            SQUARE_LENGTH_MM / 1000.0,
            MARKER_LENGTH_MM / 1000.0,
            dictionary,
        )
    return cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH_MM / 1000.0,
        MARKER_LENGTH_MM / 1000.0,
        dictionary,
    )


def main():
    board = make_board()
    board_w_mm = SQUARES_X * SQUARE_LENGTH_MM
    board_h_mm = SQUARES_Y * SQUARE_LENGTH_MM
    size_px = (
        round(board_w_mm * RENDER_DPI / 25.4),
        round(board_h_mm * RENDER_DPI / 25.4),
    )
    if hasattr(board, "generateImage"):
        image = board.generateImage(size_px, marginSize=0, borderBits=1)
    else:
        image = board.draw(size_px, marginSize=0, borderBits=1)
    png = BytesIO()
    Image.fromarray(image).save(png, format="PNG")
    png.seek(0)

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    page_w, page_h = A4
    pdf = canvas.Canvas(str(OUTPUT), pagesize=A4, pageCompression=1)
    pdf.setTitle("ArmPi FPV eye-to-hand ChArUco board - 6x4, 20 mm")
    pdf.setAuthor("ArmPi FPV calibration tools")

    pdf.setFont("Helvetica-Bold", 13)
    pdf.drawCentredString(page_w / 2, page_h - 21 * mm, "ArmPi FPV - Eye-to-Hand Calibration Target")
    pdf.setFont("Helvetica", 9)
    pdf.drawCentredString(
        page_w / 2,
        page_h - 29 * mm,
        "6 x 4 squares  |  20 mm square  |  15 mm marker  |  DICT_4X4_50",
    )

    board_x = (page_w - board_w_mm * mm) / 2
    board_y = 150 * mm
    pdf.drawImage(
        ImageReader(png),
        board_x,
        board_y,
        width=board_w_mm * mm,
        height=board_h_mm * mm,
        preserveAspectRatio=False,
        mask="auto",
    )

    # This is a cutting guide, not part of the target geometry.
    pdf.setStrokeColorRGB(0.60, 0.60, 0.60)
    pdf.setLineWidth(0.35)
    pdf.rect(board_x - 5 * mm, board_y - 5 * mm, 130 * mm, 90 * mm)
    pdf.setFont("Helvetica", 8)
    pdf.setFillColorRGB(0.32, 0.32, 0.32)
    pdf.drawCentredString(page_w / 2, board_y - 10 * mm, "Cut on grey outline; keep the full black-and-white pattern.")

    # A physical length check catches printer scaling and page-fit settings.
    ref_x = (page_w - 100 * mm) / 2
    ref_y = 107 * mm
    pdf.setStrokeColorRGB(0, 0, 0)
    pdf.setLineWidth(0.8)
    pdf.line(ref_x, ref_y, ref_x + 100 * mm, ref_y)
    for x in (ref_x, ref_x + 100 * mm):
        pdf.line(x, ref_y - 2 * mm, x, ref_y + 2 * mm)
    pdf.setFillColorRGB(0, 0, 0)
    pdf.setFont("Helvetica-Bold", 9)
    pdf.drawCentredString(page_w / 2, ref_y - 7 * mm, "100 mm PRINT-SCALE CHECK")

    pdf.setFont("Helvetica", 9)
    instructions = (
        "Print on A4 at 100% / Actual size. Disable Fit to page.",
        "Measure the line above: it must be 100 mm.",
        "Glue the cut target to a flat, lightweight rigid plate; do not bend it.",
    )
    for index, line in enumerate(instructions):
        pdf.drawCentredString(page_w / 2, (78 - 7 * index) * mm, line)

    pdf.setFont("Helvetica", 8)
    pdf.drawCentredString(
        page_w / 2,
        32 * mm,
        "Calibration parameters: squaresX=6, squaresY=4, squareLength=0.020 m, markerLength=0.015 m",
    )
    pdf.drawCentredString(page_w / 2, 27 * mm, "Dictionary: DICT_4X4_50  |  Target artwork: 120 x 80 mm")
    pdf.showPage()
    pdf.save()
    print(OUTPUT)


if __name__ == "__main__":
    main()
