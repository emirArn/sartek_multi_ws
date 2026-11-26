from setuptools import find_packages, setup
from pathlib import Path
import os

package_name = 'sartek_sim'
pkg_dir = Path(__file__).parent


def collect_files(rel_dir: str):
    """Install rel_dir altındaki tüm dosyaları dizin yapısını koruyarak share altına kopyalar."""
    data = []
    base = pkg_dir / rel_dir
    if not base.exists():
        return data

    for path in base.rglob('*'):
        if path.is_file():
            rel_parent = path.parent.relative_to(pkg_dir)
            install_dir = os.path.join('share', package_name, str(rel_parent))
            data.append((install_dir, [str(path.relative_to(pkg_dir))]))
    return data


data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    (f'share/{package_name}', ['package.xml']),
]

data_files += collect_files('launch')
data_files += collect_files('worlds')
data_files += collect_files('models')

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mert',
    maintainer_email='mert@todo.todo',
    description='SARTEK Savaşan İHA Gazebo Harmonic ve PX4 Micro XRCE-DDS simülasyon paketi',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'virtual_uav_runner = sartek_sim.virtual_uav_runner:main',
        ],
    },
)
