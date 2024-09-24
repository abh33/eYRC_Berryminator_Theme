# -*- mode: python ; coding: utf-8 -*-


block_cipher = None


a = Analysis(['test_task_4.py'],
             pathex=['F:\\e-Yantra-2020-22\\HB_2_0\\Task_4\\evaluation\\final_evaluation','F:\\Miniconda_3\\Installation\\envs\\pyinstaller_exe\\Lib\\site-packages\\cv2'],
             binaries=[('F:\\e-Yantra-2020-22\\HB_2_0\\Task_4\\evaluation\\final_evaluation\\libiconv.dll', '.'), ('F:\\e-Yantra-2020-22\\HB_2_0\\Task_4\\evaluation\\final_evaluation\\libzbar-64.dll', '.'), ('F:\\e-Yantra-2020-22\\HB_2_0\\Task_4\\evaluation\\final_evaluation\\sim.py', '.'), ('F:\\e-Yantra-2020-22\\HB_2_0\\Task_4\\evaluation\\final_evaluation\\simConst.py', '.'), ('F:\\e-Yantra-2020-22\\HB_2_0\\Task_4\\evaluation\\final_evaluation\\remoteApi.dll', '.')],
             datas=[('F:\\e-Yantra-2020-22\\HB_2_0\\Task_4\\evaluation\\final_evaluation\\task_4_dummy.ttm', '.'), ('F:\\e-Yantra-2020-22\\HB_2_0\\Task_4\\evaluation\\final_evaluation\\BM_Bot_general.ttm', '.')],
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
          name='test_task_4',
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
