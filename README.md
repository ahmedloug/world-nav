# world-nav

Latent world model for predictive collision avoidance on a Unitree Go2 quadruped, trained on simulated construction sites (Isaac Lab) with a frozen V-JEPA 2.1 encoder. Training and offline evaluation are in [`go2_wm_vjepa21.ipynb`](go2_wm_vjepa21.ipynb).

## Demo: world model shield vs naive

Here is an example of the **world model shield** compared with a **naive** controller on the same scene (seed 90012, dozer). Both robots start at **S** and must reach the goal **G**.

- **Top: naive.** It follows the straight route to the goal (dashed line in the top view).
- **Bottom: WM shield.** At each step the world model imagines the future of the candidate commands from the front camera, and the shield steers away when the imagined risk is high.

Each row shows the chase camera, the Go2 front camera (the model's input) and a top view with the trail and the plan. The header shows the current command, the clearance to the machine, and for the shield the imagined risk and the current danger.

![World model shield vs naive (seed 90012)](assets/demo_seed12_shield_vs_naive.gif)

*Full-quality video: [`assets/seed12-wmvsnaive.mp4`](assets/seed12-wmvsnaive.mp4).*
