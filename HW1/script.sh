#!/bin/bash

# 1. 生成目录结构及目录下的文件
mkdir -p linux_practice/name
mkdir -p linux_practice/permission
touch linux_practice/name/file1.txt
touch linux_practice/name/file2.txt
touch linux_practice/permission/file3.txt
touch linux_practice/permission/file4.txt

# 2. 删除name目录下的file1.txt文件
rm -f linux_practice/name/file1.txt

# 3. 将name目录下file2.txt文件的名字修改为show.txt
mv linux_practice/name/file2.txt linux_practice/name/show.txt

# 4. 修改show.txt文件的内容，第一行内容为"Hello linux"
echo "Hello linux" > linux_practice/name/show.txt

# 5. 输出show.txt文件的内容（通过cat方式）
cat linux_practice/name/show.txt

# 6. 修改permission目录下的文件权限为"-rw-r--r--"，并输出信息
for file in linux_practice/permission/file3.txt linux_practice/permission/file4.txt; do
    chmod 644 "$file"
    echo "Changed permissions for $(basename "$file") to -rw-r--r--"
done

