# -*- mode: python ; coding: utf-8 -*-


block_cipher = None


a = Analysis(['BM_GUI.py'],
             pathex=['C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder'],
             binaries=[('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\task_5_cardinal.exe', '.'),
			           ('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\libiconv.dll', '.'),
					   ('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\libzbar-64.dll', '.'),
					   ('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\remoteApi.dll', '.'),
					   ('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\sim.py', '.'),
					   ('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\simConst.py', '.')],
             datas=[('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\berryminator.png', '.'),
					('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\berryminator_theme.png', '.'),
					('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\robotics_comp.png', '.'),
			        ('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\logo_eyantra.png', '.'),
					('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\blueberry.png', '.'),
					('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\strawberry.png', '.'),
					('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\lemon.png', '.'),
					('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\team_bm.csv', '.'),
					('C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\Theme_Config.json', '.')],
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
          name='BM_GUI',
          debug=False,
          bootloader_ignore_signals=False,
          strip=False,
          upx=True,
          upx_exclude=[],
          runtime_tmpdir=None,
          console=False,
		  icon='C:\\Users\\abhin\\Desktop\\Workspace\\Berryminator GUI Testing\\Task 4 - GUI Folder\\robot12.ico',
          disable_windowed_traceback=False,
          target_arch=None,
          codesign_identity=None,
          entitlements_file=None )
