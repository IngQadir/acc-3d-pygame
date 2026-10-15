"""
make_demo_gif.py - render a scripted ACC scenario to demo.gif (no window needed).

Scenario: cruise ON at 120 km/h -> slower car ahead -> ACC slows down and follows
-> lead car blinks and changes lane -> ego resumes 120 km/h.

Usage:  pip install pillow
        python make_demo_gif.py
"""
import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame                                   # noqa: E402
from PIL import Image                           # noqa: E402

import acc_3d                                   # noqa: E402
from acc_model import EGO_LANE, KMH, Car, Inputs, World  # noqa: E402

SIM_DT = 1 / 60
DURATION = 22.0          # s of simulated time
GIF_FPS = 12
SCALE = 0.45             # output size relative to the 1200x760 window
OUT = "demo.gif"


def main():
    pygame.init()
    screen = pygame.display.set_mode((acc_3d.WIDTH, acc_3d.HEIGHT))
    renderer = acc_3d.Renderer(screen)

    world = World(seed=3, traffic=False)        # scripted: only the cars we add
    world.ego.speed = 120 * KMH
    world.step(SIM_DT, Inputs(cruise_pressed=True))
    lead = Car(world.ego.pos + 110, EGO_LANE, 80 * KMH, (200, 50, 50), is_lead=True)
    lead.change_after = 5.0
    world.traffic.append(lead)
    world.traffic.append(Car(world.ego.pos + 60, EGO_LANE + 1, 95 * KMH, (235, 200, 60)))

    size = (int(acc_3d.WIDTH * SCALE), int(acc_3d.HEIGHT * SCALE))
    every = round(1 / (GIF_FPS * SIM_DT))
    frames = []
    for i in range(int(DURATION / SIM_DT)):
        world.step(SIM_DT, Inputs())
        if i % every == 0:
            renderer.draw(world, {"CRUISE": False})
            raw = pygame.image.tobytes(screen, "RGB")
            img = Image.frombytes("RGB", screen.get_size(), raw).resize(size, Image.LANCZOS)
            frames.append(img.quantize(colors=64, method=Image.Quantize.MEDIANCUT))

    frames[0].save(OUT, save_all=True, append_images=frames[1:],
                   duration=int(1000 / GIF_FPS), loop=0, optimize=True)
    print(f"saved {OUT}: {len(frames)} frames, {os.path.getsize(OUT) / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
