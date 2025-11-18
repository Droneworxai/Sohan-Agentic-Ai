# 🤖 Robotic Mission planning with agentic ai concepts

Autonomous agricultural robot with LLM-powered decision making for optimal field treatment.

## 🎯 Features

- **LLM-Powered Agent**: Uses Groq/LLaMA to analyze environmental conditions and make intelligent routing decisions
- **Adaptive Behavior**: Makes different decisions based on time of day, weather, wind direction, and battery constraints
- **ROS2 Integration**: Full autonomous navigation with waypoint following
- **Real-time Visualization**: Live path tracking and status monitoring
- **GeoJSON Support**: Ingests real farm field coordinates

## 🏗️ Architecture
```
┌─────────────────────────────────────────────────┐
│           LLM Agent (Decision Layer)             │
│  Analyzes: Sun, Wind, Battery, Plant Stress     │
│  Decides: Clockwise vs Counter-Clockwise        │
└─────────────────┬───────────────────────────────┘
                  │ mission_plan.json
                  ▼
┌─────────────────────────────────────────────────┐
│      ROS2 Navigation (Execution Layer)          │
│  • robot_simulator.py - Robot physics           │
│  • waypoint_follower.py - Path controller       │
└─────────────────────────────────────────────────┘
```

## 📂 Project Structure
```
lavender_agentic_robot/
├── src/
│   ├── ros2_nodes/          # ROS2 navigation nodes
│   └── llm_agent/           # LLM decision making
├── data/
│   ├── geojson/             # Field coordinate data
│   └── mission_plan.json    # Current mission (modified by LLM)
├── visualization/           # Real-time visualization
└── tests/                   # LLM decision testing
```

## 🚀 Quick Start

### Prerequisites
- ROS2 Humble
- Python 3.10+
- Groq API key

### Installation
```bash
pip install groq rclpy numpy
export GROQ_API_KEY="your_key_here"
```

### Running the System

**Terminal 1 - Robot Simulator:**
```bash
cd ~/lavender_robot_ws
source install/setup.bash
ros2 run lavender_bot robot_simulator
```

**Terminal 2 - LLM Agent:**
```bash
cd ~/lavender_agentic_robot
python3 src/llm_agent/llm_direction_agent.py
```

**Terminal 3 - Navigation:**
```bash
cd ~/lavender_robot_ws
source install/setup.bash
ros2 run lavender_bot waypoint_follower
```

**Terminal 4 - Visualization:**
```bash
cd ~/lavender_agentic_robot
python3 visualization/viz_ros2.py
```

## 🎯 How It Works

1. **LLM Agent** analyzes conditions (time, wind, sun)
2. **Decides** optimal direction (clockwise/counter-clockwise)
3. **Modifies** mission_plan.json
4. **Robot** loads mission and navigates accordingly
5. **Different conditions** → **Different behavior**

## 📊 Example Output
```
✅ LLM DECISION: COUNTER-CLOCKWISE
   Confidence: high

📋 REASONING:
   Afternoon sun creates glare. Counter-clockwise
   minimizes camera interference and optimizes
   treatment effectiveness.
```

## 🔬 Key Innovation

True agentic behavior - LLM reasons about tradeoffs and provides natural language explanations for decisions that directly affect robot behavior.

## 📄 License

MIT
