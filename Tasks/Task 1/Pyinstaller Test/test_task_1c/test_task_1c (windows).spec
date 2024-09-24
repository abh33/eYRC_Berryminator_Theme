# -*- mode: python ; coding: utf-8 -*-


block_cipher = None


a = Analysis(['test_task1_new.py'],
             pathex=['C:\\Users\\Platform Team\\Desktop\\Harman\\Harvester_Bot_2-Tasks\\Harvester_Bot_2-Tasks\\PyInstaller'
			,'C:\\Users\\Platform Team\\miniconda3\\envs\\py39\\lib\\site-packages\\cv2'],
             binaries=[('C:\\Users\\Platform Team\\Desktop\\Harman\\Harvester_Bot_2-Tasks\\Harvester_Bot_2-Tasks\\PyInstaller\\libiconv.dll','.'),
			('C:\\Users\\Platform Team\\Desktop\\Harman\\Harvester_Bot_2-Tasks\\Harvester_Bot_2-Tasks\\PyInstaller\\libzbar-64.dll','.')],
             datas=[('C:\\Users\\Platform Team\\Desktop\\Harman\\Harvester_Bot_2-Tasks\\Harvester_Bot_2-Tasks\\PyInstaller\\dummy_for_1c.ttm','.')],
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
          name='test_task1_new',
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
