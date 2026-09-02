# Security and safety

Please report software vulnerabilities through GitHub's private vulnerability
reporting feature rather than a public issue.

This research project is not safety-certified. Neural inference, USB cameras,
ROS middleware, and general-purpose operating systems can drop, delay, or
misclassify data. Integrators must provide independent risk reduction,
watchdogs, safe-state behavior, and certified emergency-stop hardware where
required. A stale or missing topic must never be interpreted as proof that an
area is safe.
