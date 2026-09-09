"""将内置的 CPython 3.12 厂家扩展标记为平台相关二进制。"""

from setuptools import Distribution, setup


class BinaryDistribution(Distribution):
    """确保 wheel 不会被错误标记成跨平台的 py3-none-any。"""

    def has_ext_modules(self) -> bool:
        return True


setup(distclass=BinaryDistribution)
