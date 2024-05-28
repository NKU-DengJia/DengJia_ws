import xml.etree.ElementTree as ET
from pyproj import Transformer

# 定义输入输出文件
osm_file = 'input.xml'
output_file = 'output.txt'

# 定义墨卡托投影
transformer = Transformer.from_crs('epsg:4326','epsg:3857')

# 解析OSM文件
tree = ET.parse(osm_file)
root = tree.getroot()

# 存储节点信息的字典
nodes = {}
# 第一个节点作为参考点
reference_node = None

# 提取所有节点的信息
for node in root.findall('node'):
    node_id = node.get('id')
    lat = float(node.get('lat'))
    lon = float(node.get('lon'))
    nodes[node_id] = (lat, lon)
    if reference_node is None:
        reference_node = (lat, lon)

print("Reference node:", reference_node)

# 打开输出文件
with open(output_file, 'w') as f:
    # 遍历所有节点
    for node_id, (lat, lon) in nodes.items():
        # 转换坐标
        x, y = transformer.transform(lat, lon)
        print("Original coordinates:", lat, lon)
        print("Transformed coordinates:", x, y)
        # 第一个点作为参考点
        x_reference, y_reference = transformer.transform(reference_node[0], reference_node[1])
        # 调整坐标
        x_adjusted = x - x_reference
        y_adjusted = y - y_reference
        print("Adjusted coordinates:", x_adjusted, y_adjusted)
        print("--------------------------------")
        # 写入输出文件
        f.write(f"{x_adjusted} {y_adjusted}\n")

print("转换完成！结果已保存到 output.txt")

