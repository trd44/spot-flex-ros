import argparse
import os
from pathlib import Path

from bosdyn.client import create_standard_sdk
from bosdyn.client.graph_nav import GraphNavClient


def _load_repo_env() -> None:
    env_path = Path('/repo/.env')
    if not env_path.exists():
        return
    for line in env_path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith('#') or '=' not in line:
            continue
        key, value = line.split('=', 1)
        os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


def _credentials(args):
    _load_repo_env()
    hostname = args.hostname or os.environ.get('SPOT_IP') or os.environ.get('BOSDYN_CLIENT_HOSTNAME')
    username = (
        args.username
        or os.environ.get('BOSDYN_CLIENT_USERNAME')
        or os.environ.get('SPOT_USERNAME')
    )
    password = (
        args.password
        or os.environ.get('BOSDYN_CLIENT_PASSWORD')
        or os.environ.get('SPOT_PASSWORD')
    )
    missing = [
        name for name, value in (
            ('hostname/SPOT_IP', hostname),
            ('username/SPOT_USERNAME', username),
            ('password/SPOT_PASSWORD', password),
        )
        if not value
    ]
    if missing:
        raise RuntimeError(f'missing credentials: {", ".join(missing)}')
    return hostname, username, password


def _write_bytes(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def _download_graph(graph_client: GraphNavClient, output_dir: Path, download_images: bool) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    graph = graph_client.download_graph()
    if graph is None:
        raise RuntimeError('robot returned an empty GraphNav graph')

    _write_bytes(output_dir / 'graph', graph.SerializeToString())

    for waypoint in graph.waypoints:
        if not waypoint.snapshot_id:
            continue
        snapshot = graph_client.download_waypoint_snapshot(
            waypoint.snapshot_id,
            download_images=download_images,
        )
        _write_bytes(
            output_dir / 'waypoint_snapshots' / waypoint.snapshot_id,
            snapshot.SerializeToString(),
        )

    for edge in graph.edges:
        if not edge.snapshot_id:
            continue
        snapshot = graph_client.download_edge_snapshot(edge.snapshot_id)
        _write_bytes(
            output_dir / 'edge_snapshots' / edge.snapshot_id,
            snapshot.SerializeToString(),
        )

    print(f'Downloaded {len(graph.waypoints)} waypoints and {len(graph.edges)} edges to {output_dir}')
    if graph.waypoints:
        print('Waypoints:')
        for waypoint in graph.waypoints:
            name = waypoint.annotations.name
            label = f'{name} ({waypoint.id})' if name else waypoint.id
            print(f'  {label}')


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(
        description='Download the GraphNav map currently loaded on Spot to a local .walk folder.',
    )
    parser.add_argument(
        '--output',
        default='/repo/workspace/maps/demo.walk',
        help='Output .walk directory to write in the container',
    )
    parser.add_argument('--hostname', help='Spot hostname/IP; defaults to SPOT_IP from /repo/.env')
    parser.add_argument('--username', help='Spot username; defaults to BOSDYN_CLIENT_USERNAME or SPOT_USERNAME')
    parser.add_argument('--password', help='Spot password; defaults to BOSDYN_CLIENT_PASSWORD or SPOT_PASSWORD')
    parser.add_argument('--download-images', action='store_true', help='Include full waypoint images in the download')
    parser.add_argument('--force', action='store_true', help='Allow writing into a non-empty output directory')
    args = parser.parse_args(argv)

    output_dir = Path(args.output).expanduser()
    if output_dir.exists() and any(output_dir.iterdir()) and not args.force:
        raise RuntimeError(f'{output_dir} is not empty; pass --force or choose a new output path')

    hostname, username, password = _credentials(args)
    sdk = create_standard_sdk('spot-flex-graphnav-downloader')
    robot = sdk.create_robot(hostname)
    robot.authenticate(username, password)
    robot.time_sync.wait_for_sync()

    graph_client = robot.ensure_client(GraphNavClient.default_service_name)
    _download_graph(graph_client, output_dir, args.download_images)


if __name__ == '__main__':
    main()
