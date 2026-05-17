# Security Policy

## Project status

OpenAMR Platform Software is currently experimental and under active development.

The software may affect simulation behavior and, in future versions, real robot behavior. Security and safety issues should be reported responsibly.

---

# Supported versions

At this stage, only the latest `main` branch is actively maintained.

| Version | Supported |
| ------- | --------- |
| main    | Yes       |
| older branches | No official support |

---

# Reporting a vulnerability

If you discover a security issue, please do not disclose it publicly before the maintainers have had time to review it.

Report security concerns by contacting the OpenAMRobot maintainers through the GitHub organization or repository maintainers.

If GitHub private vulnerability reporting is enabled for this repository, use that feature.

---

# What counts as a security or safety issue

Examples include:

- unsafe robot motion caused by software behavior
- insecure communication interfaces
- exposed credentials or secrets
- unsafe default configurations
- vulnerabilities in deployment scripts
- unsafe firmware/software interaction
- unexpected motor activation
- unsafe docking behavior
- unsafe navigation behavior
- privilege escalation in tooling or scripts

---

# Safety-related issues

Because this repository may eventually control real robots, safety-related issues should be treated seriously even if they are not traditional cybersecurity vulnerabilities.

If your issue may cause:
- physical movement,
- collision risk,
- unsafe docking,
- unsafe motor behavior,
- hardware damage,

please clearly mark it as safety-related.

---

# Responsible disclosure expectations

Please include:

- affected package or file
- description of the issue
- reproduction steps
- expected behavior
- actual behavior
- possible impact
- suggested mitigation if known

---

# Secrets policy

Do not commit:

- passwords
- API keys
- private certificates
- private keys
- deployment secrets
- Wi-Fi credentials
- industrial controller credentials

If a secret is committed accidentally:
- rotate it immediately,
- remove it from repository history if necessary,
- notify maintainers if the exposure may affect others.

---

# Simulation vs real robot safety

Simulation behavior does not guarantee safe real-world robot behavior.

Before deploying to physical hardware, users are responsible for validating:

- navigation behavior
- docking behavior
- obstacle handling
- motor controller integration
- emergency stop behavior
- communication reliability
- hardware safety
- regulatory compliance

This repository is provided for research, education, and development purposes.