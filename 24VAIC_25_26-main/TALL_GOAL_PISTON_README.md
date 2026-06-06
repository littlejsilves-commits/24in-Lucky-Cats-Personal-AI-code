# Tall Goal Piston Integration - Documentation

## Changes Made

A pneumatic piston on **Port A** has been added to the skills autonomous code for tall goal scoring.

---

##  Configuration

### **Piston Declaration (Line 15):**
```cpp
digital_out TallGoalPiston = digital_out(Brain.ThreeWirePort.A);
```

### **Tall Goal ClassID (Line 30):**
```cpp
#define TALL_GOAL_ID 7  // Adjust this if your Jetson uses a different ID
```

**Note:** Change `TALL_GOAL_ID` to match your Jetson Nano's detection ID for tall goals.

---

## How It Works

### **Automatic Activation:**

1. **Robot approaches tall goal** using camera tracking
2. **When distance ≤ 0.1m (10cm):**
   - `TallGoalPiston.set(true)` - Piston extends
   - Brain screen shows: "TALL GOAL PISTON: ON"
3. **Robot scores blocks** (lever + intake sequence)
4. **After scoring:**
   - `TallGoalPiston.set(false)` - Piston retracts
   - Brain screen shows: "TALL GOAL PISTON: OFF"

---

## Piston States

| Phase | Distance | Piston State | Display |
|-------|----------|--------------|---------|
| **Searching** | > 0.1m | OFF (false) | "SEARCHING ID: 7" |
| **Approaching** | > 0.1m | OFF (false) | "Distance: 0.35 m" |
| **At Goal** | ≤ 0.1m | **ON (true)** | "TALL GOAL PISTON: ON" |
| **Scoring** | 0.1m | ON (true) | "SCORING 3 BLOCKS!" |
| **After Score** | 0.1m | OFF (false) | "TALL GOAL PISTON: OFF" |
| **Backing Away** | > 0.1m | OFF (false) | "SCORED! Goals: 1/4" |

---

## 🔧 Usage Examples

### **Example 1: Score at Tall Goal**
```cpp
// In skillsAutonomous() function, change line 285:
scoreAtGoal(TALL_GOAL_ID);  // Instead of CENTER_GOAL_ID
```

### **Example 2: Mixed Strategy (Center + Tall)**
```cpp
// Score first 2 goals at center, last 2 at tall goal
if (skillsGoalsScored < 2) {
    scoreAtGoal(CENTER_GOAL_ID);
} else {
    scoreAtGoal(TALL_GOAL_ID);
}
```

### **Example 3: Alternate Goals**
```cpp
// Alternate between center and tall goal
if (skillsGoalsScored % 2 == 0) {
    scoreAtGoal(CENTER_GOAL_ID);
} else {
    scoreAtGoal(TALL_GOAL_ID);
}
```

---

## Brain Screen Display

### **When Approaching Tall Goal:**
```
SKILLS: 3 blocks, 1 goals
Tracking ID: 7
Distance: 0.08 m
TALL GOAL PISTON: ON
```

### **When Scoring at Tall Goal:**
```
GOING TO GOAL!
SCORING 3 BLOCKS!
TALL GOAL MODE
TALL GOAL PISTON: OFF
SCORED! Goals: 2/4
```

---

## 🔌 Hardware Setup

### **Wiring:**
```
Brain 3-Wire Port A → Pneumatic Solenoid
```

### **Pneumatic System:**
- Air tank connected to solenoid
- Solenoid controls piston extension/retraction
- Port A digital output controls solenoid

---

##  Important Notes

### **1. ClassID Configuration**
The tall goal ClassID is set to `7` by default. **Verify this matches the Jetson configuration:**

```cpp
#define TALL_GOAL_ID 7  // Change if different
```

To check your Jetson's ClassIDs:
- View the dashboard during operation
- Check Jetson configuration files
- Common IDs: 4=Center, 5=Red, 6=Blue, 7=Tall

### **2. Piston Safety**
- Piston automatically retracts after scoring
- Piston retracts on skills completion
- Piston starts retracted at initialization

### **3. Distance Threshold**
Piston activates at **0.1m (10cm)**. To change:
```cpp
// In findAndDriveToObject(), line 73:
if (isTallGoal && distanceToTarget <= 0.15) {  // Change 0.1 to 0.15
```

---

##  Troubleshooting

### **Problem: Piston doesn't activate**
**Solutions:**
- Check Port A wiring
- Verify `TALL_GOAL_ID` matches Jetson config
- Check air pressure in tank
- Test piston manually: `TallGoalPiston.set(true);`

### **Problem: Piston activates too early**
**Solution:**
```cpp
// Reduce activation distance (line 73):
if (isTallGoal && distanceToTarget <= 0.05) {  // 5cm instead of 10cm
```

### **Problem: Piston activates too late**
**Solution:**
```cpp
// Increase activation distance (line 73):
if (isTallGoal && distanceToTarget <= 0.2) {  // 20cm instead of 10cm
```

### **Problem: Piston doesn't retract**
**Check:**
- `TallGoalPiston.set(false)` is called after scoring (line 234)
- Solenoid is functioning properly
- Air system has pressure

---

##  Code Locations

| Feature | File | Line |
|---------|------|------|
| Piston declaration | skills_autonomous.cpp | 15 |
| Tall goal ID | skills_autonomous.cpp | 30 |
| Piston activation | skills_autonomous.cpp | 73-77 |
| Piston retraction | skills_autonomous.cpp | 234-238 |
| Initialization | skills_autonomous.cpp | 260 |
| Cleanup | skills_autonomous.cpp | 291 |

---

## Features Added

**Automatic activation** - Piston extends at 0.1m from tall goal  
**Automatic retraction** - Piston retracts after scoring  
**Safety initialization** - Starts retracted  
**Safety cleanup** - Ensures retraction on completion  
**Visual feedback** - Brain screen shows piston state  
**Goal-specific** - Only activates for tall goals  
**Configurable** - Easy to adjust distance and ClassID  

---

## Integration Status

-  Piston added to skills_autonomous.cpp
-  Automatic activation logic implemented
-  Safety features included
-  Brain screen feedback added
-  Compatible with existing code
-  No changes needed to main.cpp

**The tall goal piston is ready to use!** 
