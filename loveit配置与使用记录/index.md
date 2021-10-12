# LoveIt配置与使用记录


<!--more-->
## 1. 安装
参考[LoveIt主题文档-基本概念](../loveit主题文档-基本概念)

## 2. 网站结构

使用`hugo new site my_blog`后生成的网站文件夹具有固定的结构。

### 2.1 archetypes/
存放页面`front-matter`模板，当使用：
```shell
hugo new posts/a_post.md
```
生成页面markdown文件时，即复制其中模板文件的内容。

### 2.2 assets/

本地资源存放位置。支持[本地资源引用](../loveit主题文档-内容#contents-organization).

另外，自定义css样式存放于 `assets/css/` 下。

### 2.3 static/

静态资源存放位置，存放网站图标，参考[网站图标, 浏览器配置, 网站清单](../loveit主题文档-基本概念#website-icon), 本地资源等.
支持[本地资源引用](../loveit主题文档-内容#contents-organization)，


<!-- 对比 assets 与 static -->
{{< admonition example "example - 对比 assets 与 static" >}}
- 调用 assets/image.jpg
```markdown
<!-- 可以使用 -->
![image](/image.jpg)
{{</* image src="/image.jpg" caption="image01"  width=200px */>}}  <!-- 扩展的shortcode -->
<!-- 不能使用 -->
{{</* figure src="/image.jpg" title="image01" */>}}  <!-- 内置的shortcode -->
<img src="/image.jpg"/>  <!-- HTML -->
```
- 调用 static/image.jpg
```markdown
<!-- 可以使用 -->
![image](/image.jpg)
{{</* image src="/image.jpg" caption="image01"  width=200px */>}}  <!-- 扩展的shortcode -->
{{</* figure src="/image.jpg" title="image01" */>}}  <!-- 内置的shortcode -->
<img src="/image.jpg"/>  <!-- HTML -->
```
{{< /admonition >}}

### 2.4 content/

博客文章存放于 `content/posts/`。

### 2.5 layouts/

生成网页所需的html模板、shortcode文件等。生成网页时，系统首先在`layouts/`下查找所需模板，接着去主题文件夹中的`themes/LoveIt/layouts/`下查找。因此`layouts/`下定义的html文件可以覆盖掉主题中的同名文件。

### 2.6 resources/

hugo 生成的资源文件

### 2.7 config.toml

配置文件

## 3.  [Netlify](https://www.netlify.com/) 自动发布和托管

首先将网站仓库上传到 GitHub 上， 然后登陆 netlify 创建网站即可。

{{< admonition tip>}}
如果站点有很多图片，生成站点可能会花费很多时间来生成不同尺寸的缩略图。为了加速这一过程，Hugo 会把裁剪好的图片缓存在站点目录下的 resources 文件夹。Hugo 官方建议把这个文件夹加入版本控制系统。

在Netlify中可以使用插件[hugo-cache-resources](cdeleeuwe/netlify-plugin-hugo-cache-resources)来缓存Netlify生成的resources文件夹。

在Netlify的`Plugins`中搜索该插件并安装。
{{< /admonition >}}




