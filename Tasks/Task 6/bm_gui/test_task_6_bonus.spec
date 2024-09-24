# -*- mode: python ; coding: utf-8 -*-


block_cipher = None


a = Analysis(['test_task_6_bonus.py'],
             pathex=['C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\BM_GUI_Task_6\\test_task_6'],
             binaries=[('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\BM_GUI_Task_6\\test_task_6\\libiconv.dll', '.'),
			           ('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\BM_GUI_Task_6\\test_task_6\\libzbar-64.dll', '.'),
					   ('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\BM_GUI_Task_6\\test_task_6\\sim.py', '.'),
					   ('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\BM_GUI_Task_6\\test_task_6\\simConst.py', '.'),
					   ('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\BM_GUI_Task_6\\test_task_6\\remoteApi.dll', '.')],
             datas=[('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\BM_GUI_Task_6\\test_task_6\\task_6_dummy.ttm', '.'), 
			        ('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\BM_GUI_Task_6\\test_task_6\\BM_Bot_general.ttm', '.'), 
					('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\BM_GUI_Task_6\\test_task_6\\wall_type_1.ttm', '.'), 
					('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\BM_GUI_Task_6\\test_task_6\\wall_type_2.ttm', '.'), 
					('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\BM_GUI_Task_6\\test_task_6\\wall_type_3.ttm', '.')],
             hiddenimports=[],
             hookspath=[],
             hooksconfig={},
             runtime_hooks=[],
             excludes=[],
             win_no_prefer_redirects=False,
             win_private_assemblies=False,
             cipher=block_cipher,
             noarchive=False)
pyz = PYZ(a.pure, a.zipped_data,
             cipher=block_cipher)

exe = EXE(pyz,
          a.scripts,
          a.binaries,
          a.zipfiles,
          a.datas,  
          [],
          name='test_task_6_bonus',
          debug=False,
          bootloader_ignore_signals=False,
          strip=False,
          upx=True,
          upx_exclude=[],
          runtime_tmpdir=None,
          console=True,
          disable_windowed_traceback=False,
          target_arch=None,
          codesign_identity=None,
          entitlements_file=None )
