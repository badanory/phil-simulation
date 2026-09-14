"""Helpers for preparing the exported URDF for PyBullet."""

from pathlib import Path
from typing import Tuple
from xml.etree import ElementTree as ET

PACKAGE_PREFIX = "package://drumrobot_RL_urdf/"

# ================================================================
# RUNTIME_LINK_FRAME_PATCH_POSE
# ================================================================
# 실기와 비교했을 때 arm mesh가 링크 프레임에서 뒤집히거나 살짝 밀려 보이는
# 문제를 확인하기 위해, 체크인된 URDF/STL은 건드리지 않고 runtime URDF에서만
# 링크 origin pose를 보정합니다.
#
# 중요:
# - 이 테이블은 "joint origin"을 바꾸지 않습니다.
# - 이 테이블은 현재 각 link의 visual origin과 collision origin을 둘 다
#   같은 xyz/rpy로 덮어씁니다.
# - 즉 화면에 보이는 mesh 위치와 collision 형상을 함께 옮기는 runtime patch입니다.
#
# 그래서 값이 맞지 않으면:
# - 화면만 이상해 보일 때도 visual이 같이 바뀌고
# - collision/AABB도 같이 바뀝니다.
# - joint 축 위치 자체가 틀린 경우는 이 테이블이 아니라 URDF joint origin 문제입니다.
# - 상완/하완 링크의 로컬 프레임이 둘레 방향으로 뒤집혀 있어
#   연결부 bracket이 양팔 모두 바깥을 향해 보입니다.
# - x축 180도 회전만 주면 mesh가 조인트 원점 기준으로 위아래가 뒤집혀
#   어깨/손목에서 분리돼 보일 수 있습니다.
# - 그래서 우선 "x축 180도 회전 + mesh 중심 유지용 xyz 보정"을 함께 적용해
#   연결부 방향과 체인 연속성이 같이 회복되는지 확인합니다.
#
# xyz 값은 STL bounds 기준 중심점(center)을 유지하도록 계산한 runtime용 보정입니다.
RUNTIME_LINK_FRAME_PATCH_POSE = {
    # ==========
    # left arm link origin patch
    # ==========
    "left_shoulder_1": {
        "xyz_m": (0.0, 0.0, 0.0),
        "rpy_deg": (0.0, 0.0, 0.0),
    },
    "left_shoulder_2": {
        "xyz_m": (0.0, 0.0, -0.05175),
        "rpy_deg": (180.0, 0.0, 0.0),
    },
    "left_elbow": {
        "xyz_m": (0.0, 0.0, -0.054),
        "rpy_deg": (180.0, 0.0, 0.0),
    },
    "left_wrist": {
        "xyz_m": (0.0, 0.0, -0.052),
        "rpy_deg": (180.0, 0.0, 0.0),
    },
    # ==========
    # right arm link origin patch
    # ==========
    "right_shoulder_1": {
        "xyz_m": (0.0, 0.0, 0.0),
        "rpy_deg": (0.0, 0.0, 0.0),
    },
    "right_shoulder_2": {
        "xyz_m": (0.0, 0.0, -0.04775),
        "rpy_deg": (180.0, 0.0, 0.0),
    },
    "right_elbow": {
        "xyz_m": (0.0, 0.0, -0.024),
        "rpy_deg": (180.0, 0.0, 0.0),
    },
    "right_wrist": {
        "xyz_m": (0.0, 0.0, -0.043),
        "rpy_deg": (180.0, 0.0, 0.0),
    },
}


# ================================================================
# RUNTIME_JOINT_LIMIT_PATCH
# ================================================================
# SolidWorks 내보내기 URDF는 revolute joint limit이 lower=0 upper=0
# effort=0으로 비어 있다. resetJointState 경로는 limit을 무시해서 티가
# 안 났지만, TORQUE_PHYSICS 경로에서는 PyBullet limit 솔버가 관절을
# [0,0] 범위로 끌어당겨 torque 제어와 줄다리기를 한다 (손목이 목표에
# 못 가고 낮은 각도에서 진동하는 원인). 체크인 URDF는 두고 runtime
# 복사본에서 torque 관절(손목)의 limit만 실기 값으로 보강한다.
#   - 각도: Phil-drum-robot config/motors.json min/max(-90..100 deg)를
#     production→URDF 부호로 변환한 값 (left sign +1, right sign -1)
#   - effort: 기어 순간 허용 토크 3 Nm
#   - velocity: 무부하 속도 출력단 환산 약 30.2 rad/s
RUNTIME_JOINT_LIMIT_PATCH = {
    "left_wrist": {
        "lower_deg": -90.0,
        "upper_deg": 100.0,
        "effort_nm": 3.0,
        "velocity_rad_s": 30.2,
    },
    "right_wrist": {
        "lower_deg": -100.0,
        "upper_deg": 90.0,
        "effort_nm": 3.0,
        "velocity_rad_s": 30.2,
    },
}


def build_runtime_urdf(source_urdf: Path, output_dir: Path) -> Path:
    """
    Build a runtime-only URDF that PyBullet can load directly.

    The checked-in URDF stays untouched. We only rewrite mesh paths and
    apply runtime link pose patches in a generated copy.
    """
    source_urdf = source_urdf.resolve()
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    package_root = source_urdf.parent.parent
    tree = ET.parse(source_urdf)
    root = tree.getroot()

    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith(PACKAGE_PREFIX):
            relative_path = filename[len(PACKAGE_PREFIX):]
            mesh.set("filename", str((package_root / relative_path).resolve()))

    # ==========
    # runtime link frame patch
    # ==========
    for link in root.findall("link"):
        link_name = link.get("name", "")
        patch_pose = RUNTIME_LINK_FRAME_PATCH_POSE.get(link_name)
        if patch_pose is None:
            continue

        _set_link_origin_pose(link, patch_pose["xyz_m"], patch_pose["rpy_deg"])

    # ==========
    # runtime joint limit patch (torque 관절 전용)
    # ==========
    for joint in root.findall("joint"):
        joint_name = joint.get("name", "")
        limit_patch = RUNTIME_JOINT_LIMIT_PATCH.get(joint_name)
        if limit_patch is None:
            continue

        limit_node = joint.find("limit")
        if limit_node is None:
            limit_node = ET.SubElement(joint, "limit")

        limit_node.set("lower", str(_deg_to_rad(limit_patch["lower_deg"])))
        limit_node.set("upper", str(_deg_to_rad(limit_patch["upper_deg"])))
        limit_node.set("effort", str(limit_patch["effort_nm"]))
        limit_node.set("velocity", str(limit_patch["velocity_rad_s"]))

    output_path = output_dir / f"{source_urdf.stem}.pybullet.urdf"
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path


def _deg_to_rad(angle_deg: float) -> float:
    return angle_deg * 3.141592653589793 / 180.0


def _set_link_origin_pose(
    link: ET.Element,
    xyz_m: Tuple[float, float, float],
    rpy_deg: Tuple[float, float, float],
) -> None:
    # 현재 runtime patch는 visual origin과 collision origin을 모두 같은 값으로
    # 덮어씁니다. 화면에서 보이는 STL과 collision 형상이 따로 놀지 않게 하려는
    # 의도입니다. visual만 따로 보정하고 싶다면 여기서 tag_name 처리 방식을
    # 분리해야 합니다.
    for tag_name in ("visual", "collision"):
        node = link.find(tag_name)
        if node is None:
            continue

        origin = node.find("origin")
        if origin is None:
            origin = ET.SubElement(node, "origin")
            origin.set("xyz", "0 0 0")

        origin.set("xyz", _format_xyz_m(xyz_m))
        origin.set("rpy", _format_rpy_deg_as_rad(rpy_deg))


def _format_rpy_deg_as_rad(rpy_deg: Tuple[float, float, float]) -> str:
    return " ".join(str(_deg_to_rad(angle_deg)) for angle_deg in rpy_deg)


def _format_xyz_m(xyz_m: Tuple[float, float, float]) -> str:
    return " ".join(str(axis_value) for axis_value in xyz_m)
