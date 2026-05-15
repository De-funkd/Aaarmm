# ARM CTRL v0.1
### Natural Language Robot Arm Simulator & Controller

ARM CTRL is a sophisticated, browser-based simulation of a 3-DOF (Degree of Freedom) robotic arm. It combines advanced robotics kinematics with natural language processing to create an intuitive interface for controlling robotic motion.

---

## 🚀 Key Features

- **Natural Language Control:** Powered by Gemini AI, translate plain English commands like "reach overhead" or "grab the box at waist height" into precise 3D coordinates.
- **Voice Recognition:** Hands-free operation using the Web Speech API. Hold to speak, and watch the arm respond.
- **Geometric Inverse Kinematics:** A high-performance, analytic IK solver that computes joint angles (Base, Shoulder, Elbow) in real-time.
- **Trajectory Queue:** Batch multiple commands into a sequence to create complex motion paths.
- **Advanced Visualization:**
  - **Side View (XZ Plane):** Detailed representation of the arm's reach and height.
  - **Top View (XY Plane):** Bird's-eye view of the arm's footprint.
  - **Reachability Heatmap:** Real-time visualization of the arm's workspace, highlighting optimal reach zones vs. marginal boundaries.
  - **IK Failure Analysis:** Visual indicators showing why a target might be unreachable (e.g., "Too Far", "Below Floor").

---

## 🛠️ Technical Specifications

- **Kinematics:** 3 revolute joints (Z-Y-Y configuration).
- **Link Geometry:** L1=0.4m (Base to Shoulder), L2=0.35m (Shoulder to Elbow), L3=0.25m (Elbow to End-Effector).
- **Workspace:** 1.0m radius sphere (restricted to Z >= 0).
- **Stack:** Pure Vanilla HTML5, CSS3, and JavaScript. No external dependencies or build steps required.

---

## 📖 How to Use

### 1. Setup
- Open `index.html` in a modern web browser (Chrome is recommended for Voice Input support).
- **API Key:** Obtain a free Gemini API key from [Google AI Studio](https://aistudio.google.com).
- Paste your key into the **GEMINI API KEY** field in the control panel. The status indicator will turn green (ONLINE).

### 2. Basic Control
- **Text Command:** Type a description in the **NATURAL LANGUAGE COMMAND** area and press `Enter` or click **EXECUTE**.
- **Voice Input:** Click and hold the **[ ◉ HOLD TO SPEAK ]** button, speak your command, and release.
- **Presets:** Use the **QUICK TARGETS** buttons for instant positioning to common configurations.

### 3. Trajectory Programming
- Add multiple steps to the **COMMAND QUEUE** by typing descriptions in the trajectory textarea and clicking **ADD STEP**.
- Click **RUN SEQUENCE** to execute the steps in order.
- Use **CLEAR QUEUE** to reset the trajectory.

### 4. Advanced Visualization
- **Heatmap:** Toggle the **REACHABILITY** button on the Top View panel.
  - **Green:** Optimal reach zone.
  - **Yellow:** Marginal reach (near limits).
  - **Red:** Unreachable zone.
- **Status Log:** Monitor the real-time feedback and coordinate data in the status window.

---

## 📝 Example Commands

- *"Reach forward half a meter at waist height."*
- *"Extend fully upward."*
- *"Move to the right at chest height."*
- *"Reach low and far forward."*
- *"Return to center at 0.5m height."*

---

## 📂 Project Structure

- `index.html`: The core application containing the UI, Styles, Kinematics Solver, and AI Integration.
- `README.md`: Documentation and usage guide.

---

*Built for the Activate VC Fellows challenge.*
