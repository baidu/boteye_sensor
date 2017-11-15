
目录结构

├── README.txt
├── BOM                                //元件描述BOM
│   ├── XPCAM-C 1V4 PCB BOM.xls       //XP3副板PCB物料
│   ├── XPIAC-C 1V3 PCB BOM.xls       //XP3主板PCB物料
│
├── PCB file                           //PCB文件（可用DXP打开、编译）
│   ├── XPCAM-C_1V4.PCBDOC            //XP3副板PCB文件
│   ├── XPIAC-C_1V3.PCBDOC            //XP3主板PCB文件
│
├── schematic                          //原理文件（可用DXP打开、编译）
│   ├── XPCAM-C_1V4                    //XP3副板原理图文件
│   │   ├── sch_Camera.SchDoc                 
│   │   ├── XPCAM-C_1V4.PrjPCB                  
│
│   ├── XPIAC-C_1V3                  //XP3主板原理图文件   
│   │   ├── sch_Camera.SchDoc 
│   │   ├── sch_Connet&IMU.SchDoc                 
│   │   ├── sch_CPU.SchDoc    
│   │   ├── sch_Power.SchDoc                
│   │   ├── sch_Top.SchDoc     
│   │   ├── sch_USB3.0.SchDoc                
│   │   ├── XPIAC-C_1V3.PrjPCB   
│
├── schematic&PCB PDF                //原理图、PCB的pdf格式
│   ├── PCB                         //XP3 PCB文件pdf格式
│   │   ├── XPCAM-C_1V4.pdf                
│   │   ├── XPIAC-C_1V3.pdf   
│
│   ├── schematic                  //XP3 原理图文件pdf格式
│   │   ├── XPCAM-C_1V4_SCH.pdf                
│   │   ├── XPIAC-C_1V3_SCH.pdf      
└── 

硬件版本说明：

1.优化XPCAM电源
2.将XPIMU与XPCAM合并为XPIAC
3.兼容彩色跟黑白
