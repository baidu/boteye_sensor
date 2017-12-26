
目录结构

├── README.txt
├── BOM                              //元件描述BOM
│   ├── XPCAM 1V1 PCB BOM.xls       //XP副板PCB物料
│   ├── XPIMU 1V1 PCB BOM.xls       //XP主板PCB物料
│
├── PCB file                         //PCB文件（可用DXP打开、编译）
│   ├── XPCAM_1V1.PCBDOC            //XP副板PCB文件
│   ├── XPIMU_1V1.PCBDOC            //XP主板PCB文件
│
├── schematic                        //原理文件（可用DXP打开、编译）
│   ├── XPCAM_1V1                  //XP副板原理图文件
│   │   ├── sch_Camera.SchDoc                 
│   │   ├── XPCAM_1V1.PrjPCB                  
│
│   ├── XPIMU_1V1                  //XP主板原理图文件   
│   │   ├── sch_Connet&IMU.SchDoc                 
│   │   ├── sch_CPU.SchDoc    
│   │   ├── sch_Power.SchDoc                
│   │   ├── sch_Top.SchDoc     
│   │   ├── sch_USB3.0.SchDoc                
│   │   ├── XPIMU_1V1.PrjPCB   
│
├── schematic&PCB PDF                //原理图、PCB的pdf格式
│   ├── PCB                         //XP PCB文件pdf格式
│   │   ├── XPCAM_1V1.pdf                
│   │   ├── XPIMU_1V1.pdf   
│
│   ├── schematic                  //XP 原理图文件pdf格式
│   │   ├── XPCAM_1V1_SCH.pdf                
│   │   ├── XPIMU_1V1_SCH.pdf      
└── 

硬件版本说明：

1.第一版，为主板连接双sensor的形式。
