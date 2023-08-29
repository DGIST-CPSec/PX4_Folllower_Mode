log_data = """
header: 
  seq: 8311
  stamp: 
    secs: 1820
    nsecs: 604000887
  frame_id: "map"
pose: 
  position: 
    x: 0.03610597550868988
    y: 0.04417268931865692
    z: 1.9895342588424683
  orientation: 
    x: -0.0320897978122718
    y: -0.007463183493113862
    z: 0.00025962458682715584
    w: -0.9994571314769086
header: 
  seq: 8313
  stamp: 
    secs: 1820
    nsecs: 676000864
  frame_id: "map"
pose: 
  position: 
    x: 0.03509332612156868
    y: 0.04320169612765312
    z: 1.9900991916656494
  orientation: 
    x: -0.03213775434043212
    y: -0.007337221050893978
    z: 0.0003101165113432165
    w: -0.9994565414210295
header: 
  seq: 8314
  stamp: 
    secs: 1820
    nsecs: 708000864
  frame_id: "map"
pose: 
  position: 
    x: 0.03452833741903305
    y: 0.04260750487446785
    z: 1.9902960062026978
  orientation: 
    x: -0.03213596968481549
    y: -0.007253147307737854
    z: 0.00030122352630740324
    w: -0.9994571736237572
"""

# 로그 데이터를 라인별로 분리하여 리스트 생성
log_lines = log_data.strip().split('\n')

# nsecs 값 추출하여 리스트에 저장
nsecs_list = [int(line.split(":")[-1].strip()) for line in log_lines if line.startswith("    nsecs")]

# 평균 계산
average_nsecs = sum(nsecs_list) / len(nsecs_list)

# 평균 격차 계산
deviations = [abs(nsecs - average_nsecs) for nsecs in nsecs_list]
average_deviation = sum(deviations) / len(deviations)

print("Average deviation:", average_deviation)