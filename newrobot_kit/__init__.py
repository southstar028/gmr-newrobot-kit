"""gmr-newrobot-kit: helpers to add a new robot to GMR retargeting.

Modules:
- quat_offset: compute quat_offset via G1 baseline + zero-pose FK delta.
- scale_table: compute initial human_scale_table via distance ratio.
"""
__all__ = ["quat_offset", "scale_table"]
__version__ = "0.1.0"
