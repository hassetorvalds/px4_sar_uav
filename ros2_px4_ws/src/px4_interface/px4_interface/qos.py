"""PX4 uXRCE-DDS 话题的 QoS 约定。

PX4 通过 uXRCE-DDS 发布/订阅的话题使用 BEST_EFFORT + TRANSIENT_LOCAL。
用默认的 RELIABLE QoS 订阅会得到 "offering incompatible QoS"，
表现为话题存在但收不到数据。所有与 /fmu/* 通信的节点都必须用这里的 QoS。

注意：本模块只依赖 rclpy.qos，不导入 px4_msgs，便于单独做单元测试。
"""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

#: 与 PX4 话题匹配的 QoS 深度
PX4_QOS_DEPTH = 10


def px4_qos(depth: int = PX4_QOS_DEPTH) -> QoSProfile:
    """返回与 PX4 uXRCE-DDS 话题兼容的 QoS 配置。

    PX4 的发布端（/fmu/out/*）与订阅端（/fmu/in/*）都是
    BEST_EFFORT + TRANSIENT_LOCAL，所以收发两端使用同一份配置。
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
    )
