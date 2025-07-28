import torch
from torch import nn
from torchvision import transforms,datasets
from torch.utils.data.dataloader import DataLoader
import torch.optim as optim
import torch.nn.functional as F
from torchinfo import summary
import os

class mixed_net(nn.Module):
    def __init__(self):
        super(mixed_net,self).__init__()

        # 第1层：卷积层 - 从Transforms.resize的64*64彩色图片(3通道)提取32个特征
        # 输入: (batch, 3, 64, 64) → 输出: (batch, 32, 62, 62)
        self.conv1 = nn.Conv2d(3, 32, kernel_size=3, stride=1, padding=0)
        # 解释：3是输入通道(RGB)，32是输出通道(特征数量)，3x3是卷积核大小
        # 为什么是 (64 - 3) / 1 + 1 = 62: 
        # 64长度-3卷积核长度=能移动的长度61，61/1步长=步数61，+1得到输出长度62
        # 所谓“卷积核组数=输出通道数”：
        # 输入通道3，则每组卷积核有3个，分别提取R、G、B通道特征
        # 输出通道是人为规定的超参数，想要32个特征，就设置32个卷积核组(如何选择？)

        # BatchNorm归一化层 - 输入: (batch, 32, 62, 62) -> 输出: (batch, 32, 62, 62)
        # 对32该通道的每个特征图进行标准化，使得每个特征图的均值为0，方差为1
        # 常跟在卷积层后面，这有助于加速训练和提高模型稳定性
        self.bn1 = nn.BatchNorm2d(32)

        # 池化层：一种降采样层，缩小图片尺寸，减少计算量
        # 输入: (batch, 32, 62, 62) → 输出: (batch, 32, 31, 31)
        # 超参数：窗口尺寸(kernel_size)和步长(stride)，一般二者相同
        # 窗口尺寸一般设置为2x2
        self.pool1 = nn.MaxPool2d(kernel_size=2, stride=2)
        # MaxPooling：取窗口内最大值，保留最强特征
        # 对于特征检测任务，MaxPooling能保留最强的激活，效果更好
        # 例如：如果某个2x2区域有一个像素检测到锥桶的强信号，MaxPooling会保留这个信号
        # 而AveragePooling会稀释这个信号，导致信息丢失

        # 第2层：
        # 输入: (batch, 32, 31, 31) → 输出: (batch, 64, 29, 29)
        self.conv2 = nn.Conv2d(32, 64, kernel_size=3, stride=1, padding=0)
        self.bn2 = nn.BatchNorm2d(64)
        self.pool2 = nn.MaxPool2d(kernel_size=2, stride=2)
        
        # 第3层：
        # 输入: (batch, 64, 29, 29) → 输出: (batch, 128, 27, 27)
        self.conv3 = nn.Conv2d(64, 128, kernel_size=3, stride=1, padding=0)
        self.bn3 = nn.BatchNorm2d(128)
        self.pool3 = nn.MaxPool2d(kernel_size=2, stride=2)
        
        # 第4层：
        # 输入: (batch, 128, 27, 27) → 输出: (batch, 256, 25, 25)
        self.conv4 = nn.Conv2d(128, 256, kernel_size=3, stride=1, padding=0)
        self.bn4 = nn.BatchNorm2d(256)
        
        # 自适应池化：固定输出尺寸
        self.adaptive_pool = nn.AdaptiveAvgPool2d((2, 2))
        # 尝试理解：最后一步池化为什么使用平均池化？
        # 256个通道里，每个通道池化前代表一种高级特征（如"蓝色圆形"、"红色锥形"等）
        # 每个位置代表该特征在对应区域的强度
        # 例如某个通道可能代表"蓝色桩桶特征"：
        # 蓝色特征图 = [
        #     [0.1, 0.2, 0.8, 0.3, ...],  # 第3个位置检测到强蓝色特征
        #     [0.0, 0.1, 0.7, 0.2, ...],  # 第3个位置持续强烈
        #     [0.0, 0.0, 0.6, 0.1, ...],  # 蓝色桩桶在这个区域
        #     [..., ..., ..., ..., ...]
        # ]
        # 最大池化只能知道每个通道的最大值，无法获取空间信息; 桩桶肯定不止一个像素
        # 平均池化能更好利用高特征强度区域的空间信息

        # 为什么输出2*2？
        # 2×2池化提供4个区域的特征：
        # [左上区域, 右上区域]
        # [左下区域, 右下区域]
        # 对于桩桶位置变化的任务，这很有帮助：
        # 场景1: 桩桶在左上角 → 左上区域特征强，其他区域弱
        # 场景2: 桩桶在右下角 → 右下区域特征强，其他区域弱
        # 场景3: 桩桶在中心   → 四个区域都有一定特征
        # 网络可以学会：
        # "无论桩桶在哪个区域，只要检测到蓝色+圆锥形状 → 分类为蓝色"
        # 1*1 问题：如果桩桶很小且在角落，全局平均可能会稀释特征
        # 例如：64×64图片中，桩桶只占10×10区域
        # 全局平均 = (桩桶区域强特征 + 大量背景弱特征) / 总像素数
        # 结果：桩桶特征被稀释

        # 随机关闭50%神经元，并放大剩余的神经元输出(除以0.5)，从而减少过拟合
        self.dropout = nn.Dropout(0.5)
        
        # 全连接层：做最终分类决策
        self.fc1 = nn.Linear(256 * 2 * 2, 512)  # 1024 → 512
        self.fc2 = nn.Linear(512, 128)          # 512 → 128
        self.fc3 = nn.Linear(128, 3)            # 128 → 3 (三分类)

    def forward(self, x):
        """前向传播：数据在网络中的流动过程"""
        # 卷积神经网络单层的基本结构：卷积 + 归一化 + 激活 + 池化
        # ReLU是一种常用的激活函数
        x = self.pool1(F.relu(self.bn1(self.conv1(x))))
        x = self.pool2(F.relu(self.bn2(self.conv2(x))))
        x = self.pool3(F.relu(self.bn3(self.conv3(x))))
        x = self.adaptive_pool(F.relu(self.bn4(self.conv4(x))))
        
        # 展平：从2D特征图变成1D向量
        # 全连接层接收1D向量输入，展平操作将(batch, 256, 2, 2)变为(batch, 1024)
        # x.size(0)是保留batch大小(一次处理的图片数量)，-1表示自动计算剩余维度
        x = x.view(x.size(0), -1)
        
        # 全连接层做分类
        x = self.dropout(F.relu(self.fc1(x)))
        x = self.dropout(F.relu(self.fc2(x)))
        x = self.fc3(x)  # 输出：(batch, 3)，每个样本对应3个类别的分数
        
        return x

if __name__ == "__main__":
    # 图像转换
    transforms = transforms.Compose(
        [
            transforms.Resize([64, 64]), # 将图片缩放到64x64，因此第一层卷积层的输入尺寸为64x64
            transforms.ToTensor(),# 将图片转换为Tensor格式
            transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))
        ]
    )
    
    #超参数设置
    BATCH_SIZE = 1024
    EPOCH = 200

    #加载数据
    trainset = datasets.ImageFolder(root=r'dataset/train',transform=transforms)
    testset1 = datasets.ImageFolder(root=r'dataset/test1',transform=transforms)
    testset2 = datasets.ImageFolder(root=r'dataset/test2',transform=transforms)

    print(f"训练集图片数量: {len(trainset)}")
    print(f"测试集1图片数量: {len(testset1)}")
    print(f"测试集2图片数量: {len(testset2)}")
    
    train_loader = DataLoader(trainset, batch_size=BATCH_SIZE, shuffle=True, pin_memory=True)
    test_loader1 = DataLoader(testset1, batch_size=BATCH_SIZE, shuffle=True, pin_memory=True)
    test_loader2 = DataLoader(testset2, batch_size=BATCH_SIZE, shuffle=True, pin_memory=True)

    #创建网络
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu") # NVIDIA GPU写cuda
    net = mixed_net().to(device)
    
    #打印网络信息
    summary(net, input_size=(1, 3, 64, 64), device=device)
    print(f'标签对应的ID: {trainset.class_to_idx}')

    #设置优化器、损失函数
    criterion = nn.CrossEntropyLoss()
    optimizer =optim.SGD(net.parameters(), lr=0.01, momentum=0.9)
    # optimizer = optim.Adam(net.parameters(), lr=0.001, weight_decay=1e-4)

    #开始训练

    print("Start")
    for epoch in range(EPOCH):
        train_loss = 0.0
        #print(epoch)
        
        for batch_id, (datas, labels) in enumerate(train_loader):
            datas, labels = datas.to(device), labels.to(device)

            optimizer.zero_grad()

            outputs = net(datas)

            loss = criterion(outputs, labels)

            loss.backward()

            optimizer.step()

            train_loss += loss.item()

            if epoch > 50 and (epoch + 1) % 10 == 0:
                os.makedirs("pth", exist_ok=True)
                PATH = "pth/modeltemp.pth"
                torch.save(net.state_dict(), PATH)
                model = mixed_net()
                model.load_state_dict(torch.load(PATH))
                model.eval()
                model.to(device)

                #限定保存条件
                max_correct = 99
                correct1 = 0
                correct2 = 0
                total1 = 0
                total2 = 0

                #分别测试两个数据集
                with torch.no_grad():
                    for i ,(datas1, labels1) in enumerate(test_loader1):
                        datas1, labels1 = datas1.to(device), labels1.to(device)
                        output_test1 = model(datas1)
                        _, predicted1 = torch.max(output_test1.data, dim=1)
                        total1 += predicted1.size(0)
                        correct1 += (predicted1 == labels1).sum()

                    for i ,(datas2, labels2) in enumerate(test_loader2):
                        datas2, labels2 = datas2.to(device), labels2.to(device)
                        output_test2 = model(datas2)
                        _, predicted2 = torch.max(output_test2.data, dim=1)
                        total2 += predicted2.size(0)
                        correct2 += (predicted2 == labels2).sum()

                    #打印消息
                    c1 = 0
                    c2 = 0
                    c2 = correct2 / total2 * 100
                    c1 = correct1 / total1 * 100
                    print(
                        f"epoch:{epoch + 1}\tbatch_id:{batch_id + 1}\taverage_loss:{(train_loss / len(train_loader.dataset)):.5f}\t"
                        f"correct1:{c1:.2f}%\tcorrect2:{c2:.2f}%"
                    )
                    if (c1 > max_correct):
                        max_correct = c1
                        MAX_PATH = f"pth/model_best_{max_correct}.pth"
                        print(f"save {MAX_PATH}")
                        torch.save(net.state_dict(),MAX_PATH)

