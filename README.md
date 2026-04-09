# 🚁 AI Grand Prix — Autonomous Drone Challenge

A Python-based autonomy stack for the [AI Grand Prix](https://theaigrandprix.com) global autonomous drone racing competition, organized by Anduril in partnership with DCL and Neros Technologies.

---

## 👥 Team

### Advisors
- Aneesh Saxena
- Tushar Shrivastav

### Simulation & Network Communication
- Sameer Faisal
- Yat Chun Wong
- Ryan Yang
- Trung Nguyen
  
### Algorithms & Autonomy
- Kunal Shrivastav
- Samyak Kakatur
- David Vayntrub
- Ram Rao
---

## 🏗️ Project Structure

```
aigp/
├── comms/          # MAVLink/MAVSDK communication layer
├── perception/     # Gate detection & vision pipeline
├── planning/       # Path planning & trajectory generation
├── control/        # Drone dynamics & control loop
├── main.py         # Integration entry point
└── requirements.txt
```

---

## 🛠️ Tech Stack

- **Language:** Python 3.14.2
- **Simulator:** Colosseum (Unreal Engine 5.2)
- **Communication:** MAVLink v2 via MAVSDK over UDP
- **Control Interface:** SET_POSITION_TARGET_LOCAL_NED / SET_ATTITUDE_TARGET

---

## ⚙️ Setup

```bash
# Clone the repo
git clone https://github.com/YOUR_USERNAME/YOUR_REPO_NAME.git
cd YOUR_REPO_NAME

# Create and activate virtual environment
python -m venv venv
venv\Scripts\activate  # Windows

# Install dependencies
pip install -r requirements.txt
```

### Manual flight in sim (Windows)

If the AirSim / Colosseum **Python client already connects**, teammates can add **keyboard or gamepad manual control** (vJoy, `settings.json` under **Documents** or **OneDrive\Documents**, `RemoteControlID`). Step-by-step guide:

**[docs/MANUAL_FLIGHT_WINDOWS.md](docs/MANUAL_FLIGHT_WINDOWS.md)**

---

## 🏁 Competition Overview

The AI Grand Prix is a global autonomous drone racing competition with a **$500,000 prize pool**.

| Phase | Format | Timeline |
|---|---|---|
| Virtual Qualifier | Simulated racecourse, MAVLink interface | Spring 2026 |
| In-Person Qualifier | Real drone, simulation-to-real transfer | TBD |
| Final Race | Live head-to-head | November 2026, Columbus OH |

**Rules:** No human interaction during timed runs. Fully autonomous flight only.

---

## 📡 Interface Spec

Based on VADR-TS-001 (Issue 00.01):

- Transport: UDP
- Protocol: MAVLink v2 / MAVSDK
- Physics update rate: 120 Hz
- Recommended command rate: 50–120 Hz
- No GPS — local Cartesian coordinate system only
- Max run duration: 8 minutes
