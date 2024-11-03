import rehypeShiki from '@shikijs/rehype'

export const rehypeCodeHighlight = [
  rehypeShiki,
  {
    themes: {
      light: 'github-light',
      dark: 'github-dark',
    },
    defaultColor: false,
    langs: ['c', 'cpp', 'python', 'javascript'] // 添加需要支持的语言
  },
]