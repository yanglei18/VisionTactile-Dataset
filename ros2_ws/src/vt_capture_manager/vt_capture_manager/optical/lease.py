"""Exclusive authority for local processes publishing each hand stream."""
import errno
import socket


def domain_lease(domain, service):
    # The Linux kernel releases this local abstract socket even on SIGKILL.
    lease = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        lease.bind(f"\0visiontactile.{service}.domain.{domain}")
    except OSError as exc:
        lease.close()
        if exc.errno == errno.EADDRINUSE:
            raise RuntimeError(f"ROS domain {domain} already has a local {service} process") from exc
        raise
    return lease
