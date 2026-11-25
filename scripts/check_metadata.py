#!/usr/bin/env python3
"""Check the package metadata without requiring a ROS installation."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
package = ET.parse(ROOT / "package.xml").getroot()
assert package.findtext("name") == "easynav_gradients"
assert package.findtext("license") == "Apache-2.0"
plugin = ET.parse(ROOT / "easynav_gradients_plugins.xml").getroot()
assert plugin.attrib["path"] == "easynav_gradients"
cls = plugin.find("class")
assert cls is not None
assert cls.attrib["name"] == "easynav_navmap_gradients/GradientPlanner"
assert cls.attrib["type"] == "easynav::navmap::GradientPlanner"
assert cls.attrib["base_class_type"] == "easynav::PlannerMethodBase"
params = yaml.safe_load((ROOT / "config/params.yaml").read_text())
values = params["planner_node"]["ros__parameters"]["gradients"]
expected = {
    "continuous_replan": False,
    "max_iterations": 500,
    "goal_change_epsilon": 0.001,
    "smoothing_factor": 0.4,
    "potential_drop_weight": 0.75,
    "robot_width": 0.8,
    "elastic_w_data": 0.10,
    "elastic_w_smooth": 0.35,
    "elastic_max_iterations": 15,
    "elastic_convergence_threshold": 0.002,
}
for key, value in expected.items():
    assert key in values and type(values[key]) is type(value) and values[key] == value, key
citation = yaml.safe_load((ROOT / "CITATION.cff").read_text())
assert citation["license"] == "Apache-2.0"
assert citation["version"] == package.findtext("version")
print("Package, plugin, baseline parameters and citation metadata: OK")
