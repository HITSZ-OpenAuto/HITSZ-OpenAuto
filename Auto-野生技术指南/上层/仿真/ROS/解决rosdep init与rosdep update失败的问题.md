# 解决rosdep init与rosdep update失败的问题

> 网上的教程鱼龙混杂，根本不能用，还是自己写一个吧

我们安装完ROS后，要进行如下两个操作：

```
sudo rosdep init
rosdep update
```

`rosdep`能帮助我们安装ROS包所需要的依赖，然而执行的时候，通常会出现：

![image-20230118170105143](https://git.nrs-lab.com/LiMinghang23m/picgo-pic/-/raw/main/pictures/2023/01/18_17_1_5_image-20230118170105143.png)

这个问题来源是大陆的DNS污染，我们首先要手动更改域名解析：

打开https://site.ip138.com/ ，输入`raw.githubusercontent.com`，查询其IP地址：

![image-20230118171754504](https://git.nrs-lab.com/LiMinghang23m/picgo-pic/-/raw/main/pictures/2023/01/18_17_17_54_image-20230118171754504.png)

打开一个终端，输入：

```
sudo vim /etc/hosts
```

在文件最后的末尾添加：

```
185.199.110.133 raw.githubusercontent.com
```

当然，换成`185.199.110.133`或者`185.199.109.133`都可以

保存退出，再执行：

```
rosdep init
```

如果还是访问不了的话，那我们手动进行`rosdep init`的操作：

用电脑访问https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list， 可看到如下界面：

![image-20230118170300075](https://git.nrs-lab.com/LiMinghang23m/picgo-pic/-/raw/main/pictures/2023/01/18_18_6_17_18_17_3_0_image-20230118170300075.png)

我们复制一下如下这几行，

```
# os-specific listings first
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/osx-homebrew.yaml osx

# generic
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml
yaml https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/ruby.yaml
gbpdistro https://raw.githubusercontent.com/ros/rosdistro/master/releases/fuerte.yaml fuerte

# newer distributions (Groovy, Hydro, ...) must not be listed anymore, they are being fetched from the rosdistro index.yaml instead
```

之后在终端中输入：

```
sudo mkdir -p /etc/ros/rosdep/sources.list.d/
cd /etc/ros/rosdep/sources.list.d/
sudo vim 20-default.list
```

把前面复制的内容粘贴进去，这实际上就完成了`rosdep init`的操作

之后执行：

```
rosdep update
```

通常情况下，会出现：

![image-20230118173841755](https://git.nrs-lab.com/LiMinghang23m/picgo-pic/-/raw/main/pictures/2023/01/18_17_38_41_image-20230118173841755.png)

我们使用域名代理的方法进行解决

执行：

```
sudo vim /usr/lib/python2.7/dist-packages/rosdistro/__init__.py
```

在第68行的`https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml`前面加上`https://ghproxy.com/`

![image-20230118174655958](https://git.nrs-lab.com/LiMinghang23m/picgo-pic/-/raw/main/pictures/2023/01/18_18_6_17_18_17_46_56_image-20230118174655958.png)

执行：

```
sudo vim /usr/lib/python2.7/dist-packages/rosdep2/gbpdistro_support.py
```

在第34行的`https://raw.githubusercontent.com/ros/rosdistro/`前面加上`https://ghproxy.com/`

![image-20230118175214266](https://git.nrs-lab.com/LiMinghang23m/picgo-pic/-/raw/main/pictures/2023/01/18_18_6_18_18_17_52_14_image-20230118175214266.png)

执行：

```
sudo vim /usr/lib/python2.7/dist-packages/rosdep2/sources_list.py
```

在第64行的`https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list`前面加上`https://ghproxy.com/`

![image-20230118175357488](https://git.nrs-lab.com/LiMinghang23m/picgo-pic/-/raw/main/pictures/2023/01/18_17_53_57_image-20230118175357488.png)

同一个文件，在第301行中添加：`url="https://ghproxy.com/"+url`

![image-20230118174829167](https://git.nrs-lab.com/LiMinghang23m/picgo-pic/-/raw/main/pictures/2023/01/18_18_6_19_18_18_1_15_18_17_48_29_image-20230118174829167.png)

执行：

```
sudo vim /usr/lib/python2.7/dist-packages/rosdep2/rep3.py
```

在第36行的`https://raw.githubusercontent.com/ros/rosdistro/master/releases/targets.yaml`前面加上`https://ghproxy.com/`

![image-20230118175630420](https://git.nrs-lab.com/LiMinghang23m/picgo-pic/-/raw/main/pictures/2023/01/18_17_56_30_image-20230118175630420.png)

执行：

```
sudo vim /usr/lib/python2.7/dist-packages/rosdistro/manifest_provider/github.py
```

在第68行的`https://raw.githubusercontent.com/%s/%s/package.xml`前面加上`https://ghproxy.com/`

![image-20230118175827618](https://git.nrs-lab.com/LiMinghang23m/picgo-pic/-/raw/main/pictures/2023/01/18_18_6_19_18_17_58_27_image-20230118175827618.png)

同一个文件，在第119行的`https://raw.githubusercontent.com/%s/%s/%s`前面加上`https://ghproxy.com/`

![image-20230118175927888](https://git.nrs-lab.com/LiMinghang23m/picgo-pic/-/raw/main/pictures/2023/01/18_17_59_27_image-20230118175927888.png)

之后再执行：

```
rosdep update
```

可以看到成功执行，至此ROS完美安装成功

![image-20230118180305825](https://git.nrs-lab.com/LiMinghang23m/picgo-pic/-/raw/main/pictures/2023/01/18_18_6_20_18_18_3_5_image-20230118180305825.png)
