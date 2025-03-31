import { useAtom } from 'jotai'
import { searchPanelOpenAtom } from '@/store/searchPanel'
import appConfig from '@/config.json'
import { DocSearchModal, useDocSearchKeyboardEvents } from '@docsearch/react'
import '@docsearch/css'
import { RootPortal } from './RootPortal'
import { useRef } from 'react' // 添加这一行导入 useRef

export function SearchPanel() {
  const [isOpen, setIsOpen] = useAtom(searchPanelOpenAtom)
  const searchButtonRef = useRef(null) // 添加这一行创建引用

  const onOpen = () => {
    setIsOpen(true)
  }
  const onClose = () => {
    setIsOpen(false)
  }

  useDocSearchKeyboardEvents({
    isOpen,
    onOpen,
    onClose,
    searchButtonRef, // 添加搜索按钮引用
  })

  return (
    isOpen && (
      <RootPortal>
        <DocSearchModal
          appId={appConfig.docSearch.appId}
          apiKey={appConfig.docSearch.apiKey}
          indexName={appConfig.docSearch.indexName}
          initialScrollY={window.scrollY}
          onClose={onClose}
        />
      </RootPortal>
    )
  )
}
