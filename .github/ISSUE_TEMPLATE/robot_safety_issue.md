---
name: Robot safety issue
about: Report a possible unsafe robot behavior, motion, docking, navigation, or hardware interaction
title: "[Safety]: "
labels: safety
assignees: ""
---

# Robot safety issue

Use this template for issues that may affect:

- physical robot safety
- unsafe motion
- docking risk
- navigation collisions
- hardware damage
- unsafe deployment assumptions
- unexpected actuator behavior

---

## Summary

Briefly describe the safety concern.

---

## Affected behavior

Select all that apply:

- [ ] Unexpected robot movement
- [ ] Navigation collision risk
- [ ] Unsafe docking behavior
- [ ] Motor control issue
- [ ] Sensor failure or incorrect sensor interpretation
- [ ] Hardware communication issue
- [ ] Simulation-to-real-world mismatch
- [ ] Emergency stop or safety system concern
- [ ] Power or battery-related issue
- [ ] Other

---

## Environment

- Simulation or real robot:
- OS:
- ROS 2 distribution:
- Gazebo / Gazebo Sim version:
- Robot hardware version, if applicable:
- Branch or commit:

---

## Steps to reproduce

```bash
# Paste exact commands or procedure here
```

---

## Expected safe behavior

Describe what should happen.

---

## Actual unsafe behavior

Describe what happened.

---

## Risk level

Select one:

- [ ] Low
- [ ] Medium
- [ ] High
- [ ] Critical

---

## Possible consequences

Select all that apply:

- [ ] Robot collision
- [ ] Unsafe docking
- [ ] Hardware damage
- [ ] Motor overload
- [ ] Sensor failure
- [ ] Navigation instability
- [ ] Human safety concern
- [ ] Battery or power issue
- [ ] Unknown

---

## Immediate mitigation

Describe:

- temporary workaround
- safety stop procedure
- disabled feature
- deployment limitation
- recommended precaution

---

## Logs, screenshots, or videos

Attach:

- terminal logs
- RViz screenshots
- Gazebo screenshots
- videos
- photos
- diagnostics

if possible.

---

## Additional notes

Add any additional safety context.