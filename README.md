# Fanatic

A 3D simulation where a computer-controlled fan keeps a ball floating at a target point,
and recovers it when you drag, fling or move it. The controller doesn't know the ball's
mass and has to work it out from how the ball moves.

## Setup
Requires Python 3.12.

    py -3.12 -m venv venv
    venv\Scripts\activate
    pip install -r requirements.txt

## Run
    python main.py

Left-drag the ball to grab and fling it. Right-drag to rotate the camera; scroll to zoom.

## Test
    python -m pytest

See `docs/design.md` for the architecture and design decisions.