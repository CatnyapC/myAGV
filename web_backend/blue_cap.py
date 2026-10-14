"""Measure the blue salt-can cap, including a fragment clipped at the image edge."""


def blue_cap_bounds(image):
    import cv2
    import numpy as np

    if not isinstance(image, bytes) or not 0 < len(image) <= 4 * 1024 * 1024:
        raise ValueError('Invalid blue-cap image')
    frame = cv2.imdecode(np.frombuffer(image, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise ValueError('Blue-cap image cannot be decoded')
    height, width = frame.shape[:2]
    if max(height, width) > 4096:
        raise ValueError('Blue-cap image is too large')
    # shortcut: blue salt-can caps only; use another detector for other items.
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (95, 80, 50), (135, 255, 255))
    saturated = cv2.inRange(hsv, (100, 140, 100), (135, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        # Bright saturated pixels identify the cap; the wider mask retains its printed rim.
        if (area >= width * height * .002 and .25 <= w / h <= 8 and area / (w * h) >= .2
                and cv2.countNonZero(saturated[y:y + h, x:x + w]) >= area * .35):
            candidates.append([round(x * 1000 / width), round(y * 1000 / height),
                               round((x + w) * 1000 / width), round((y + h) * 1000 / height)])
    if len(candidates) != 1:
        raise ValueError('Blue cap missing or ambiguous; inspect the camera view')
    return candidates[0]
