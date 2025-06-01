# TurtleBot3 Term Project

## 실행 안내

- 이 프로젝트를 실행하려면 터미널에서 아래 명령어를 순서대로 입력하세요.

### 사전 준비

- ROS2가 설치되어 있어야 합니다.
- Git이 설치되어 있어야 합니다.

### 단계별 명령어

1. **작업 디렉토리 생성**\
   프로젝트를 위한 새 디렉토리를 만듭니다.

   ```bash
   mkdir turtlebot3_term_pj
   ```

2. **디렉토리로 이동**\
   생성한 디렉토리로 이동합니다.

   ```bash
   cd turtlebot3_term_pj
   ```

3. **GitHub에서 프로젝트 클론**\
   GitHub 저장소에서 프로젝트를 클론합니다.

   ```bash
   git clone https://github.com/NMHK-134711/turtlebot3_term_pj.git
   ```

4. **프로ject 빌드**\
   ROS2 패키지를 빌드합니다.

   ```bash
   colcon build
   ```

5. **환경 설정**\
   ROS2 환경을 설정합니다. (필요한 경우에만 실행)

   ```bash
   source ~/.bashrc
   ```

6. **런치 파일 실행**\
   프로젝트를 시작합니다.

   ```bash
   ros2 launch turtlebot3_term_pj master_launch.py
   ```

### 추가 참고

`source ~/.bashrc`는 ROS2 환경 변수를 새로고침할 때 사용됩니다. 만약 `colcon build` 후 환경 설정이 필요하다면, `source install/setup.bash`를 대신 실행하세요.