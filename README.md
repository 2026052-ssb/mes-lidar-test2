# CAD 기반 데이터 생성

> Python 3.12

### dependency

```bash
uv sync --project mes-lidar-test
```
### PoC 순서

1. CAD → Mesh 변환

2. Ray Casting으로 LiDAR 생성

3. 여러 위치에서 생성

CAD 하나로 여러관점의 PCD를 자동 생성

4. 실제 LiDAR처럼 Noise 추가

### LiDAR 설정

LiDAR 위치, FOV, 채널 수, 해상도, 최대 거리 등은 `config/lidar.yaml`에서 관리합니다.
LiDAR 위치는 CAD 메시의 bounding box를 기준으로 계산하며, YAML의 `position` 오프셋으로 조정할 수 있습니다.

### PCD 저장

스캔한 포인트 클라우드는 기본적으로 CAD 파일명과 동일한 `.pcd` 파일로 저장됩니다.
저장 경로와 활성화 여부는 `config/lidar.yaml`의 `output`에서 설정합니다. 경로는 `main.py`의 `ROOT_PATH`를 기준으로 하며, 대상 디렉터리가 없으면 자동 생성됩니다.

### 참조

https://github.com/jonathsch/lidar-synthesis
