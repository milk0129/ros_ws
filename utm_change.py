from pyproj import Proj, transform

# WGS84 좌표계 정의 (위도, 경도)
wgs84 = Proj(init='epsg:4326')

# UTM-K 좌표계 정의
utm_k = Proj(init='epsg:5179')

# 변환할 위도와 경도
latitude = 37.63248176490206  # 예: 특정 위치의 위도
longitude = 127.05557444676354   # 예: 특정 위치의 경도

# 좌표 변환
x, y = transform(wgs84, utm_k, longitude, latitude)

print(f'UTM-K 좌표: x = {x}, y = {y}')
