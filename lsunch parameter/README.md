# 개인 스터디 2023.01.11

## 런치 파라미터 사용법 

ros2 책 p.420 ~ p.426

파라미터와 아규먼트의 차이점

 파라미터(parameter) = 매개변수

 아규먼트(arguments) = 인자 전달값

# ros2 TAML작성 법

your_node: <-- 작성한 노드 이름과 같이 맞춰줘여함

 def __init__(self, bms_id='your_node'):

``` super().__init__('your_node') <-- 요부분 ```

```
  ros__parameters: // 기본 문법 (이름은 바꿔도 됨)
    bool_value: True
    int_number: 5
    float_number: 3.14
    str_text: "Hello Universe"
    bool_array: [True, False, True]
    int_array: [10, 11, 12, 13]
    float_array: [7.5, 400.4]
    str_array: ['Nice', 'more', 'params']
    bytes_array: [0x01, 0xF1, 0xA2]
    nested_param:
      another_int: 7
```

## ===== 출력형식 ===

``` ros2 param dump /your_node--print    <-- 확인 명령어 ```

```
your_node:
  ros__parameters:
    bool_array:        // 출력 타입
    - true
    - false
    - true
    bool_value: true   // 출력 타입
    bytes_array: !!python/object/apply:array.array   // 출력 타입
    - q         // 바이트는 정수형으로 출력되는 것을 볼 수 있음 지원을 안함
    - - 1
      - 241
      - 162
    float_array: !!python/object/apply:array.array   // 출력 타입
    - d
    - - 7.5
      - 400.4
    float_number: 3.14
    int_array: !!python/object/apply:array.array   // 출력 타입
    - q
    - - 10
      - 11
      - 12
      - 13
    int_number: 5
    nested_param:
      another_int: 7
    str_array:                 // 출력 타입
    - Nice
    - more
    - params
    str_text: Hello Universe
    use_sim_time: false
```
## ============ launch 작성 예시 ============
```
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): <--- 메소드의 사용

    config = LaunchConfiguration('param_dir', default=os.path.join(
        get_package_share_directory('exploration_bot_application'),
        'config',
        'bms.yaml'))  // .yaml로 파라라미터를 가져올 수 있음
   
   return LaunchDescription([
        DeclareLaunchArgument(
        name='param_dir',
        default_value=config,
        description='Connected USB port with BMS'),
             
        // default=os.path.join 으로 파일 및 폴더를 찾아감
        // get_package_share_directory <-- 파일 경로 설정 및 파일 설정  ('~~패키지'), (폴더), (파일명.yaml)
        
        
    node=Node(
        package = 'ros2_tutorials',     // 실행할 패키지 
        name = 'your_node',             // 실행시 지정할 노트의 이름     ( 2개는 같은 이름으로 하는것을 추천 )
        executable = 'test_yaml_params', // 실제도 작동 되는 노드의 이름 ( 2개는 같은 이름으로 하는것을 추천 )
        parameters = [config]            // config = bms.yaml 안에 들어 있는 매개변수 
    )                                    // 특정 파라미터를 직접 입력해도 좋지만 DeclareLaunchArgument 에서 지정한 변수를 사용해도 좋다
```
런치 파일을 작성 했다면 setup.py의 설정을 변경해줘야 한다 런치파일을 읽을 수 있게 선언을 해줘야 하기 때문이다, config 폴더의 .yaml도 마찬가지 이다 
```
setup(
    name=pkg_name,
    version='0.0.0',
    packages=[pkg_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + pkg_name]),
        ('share/' + pkg_name, ['package.xml']),
        ('share/' + pkg_name + '/config',[   <--- .yaml 파일의 선언
            'config/bms.yaml',]),
        ('share/' + pkg_name + '/launch',[   <--- 런치 파일의 선언
            'launch/bms.launch.py',]),    
    ],
```




