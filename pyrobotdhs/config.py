logging_config = {
    'version': 1,
    'disable_existing_loggers': False,
    'propagate': True,
    'formatters': {
        'standard': {
            'format': ('%(asctime)s [%(levelname)s] %(name)s '
                       '%(filename)s:%(funcName)s:%(lineno)d | %(message)s'),
            'datefmt': '%Y-%m-%dT%H:%M:%S',
        },
        'colored': {
            '()': 'colorlog.ColoredFormatter',
            'format': ('%(log_color)s'
                       '%(asctime)s [%(levelname)s] %(name)s '
                       '%(filename)s:%(funcName)s:%(lineno)d | %(message)s'
                       '%(reset)s'),
            'datefmt': '%Y-%m-%dT%H:%M:%S',
        },
    },
    'handlers': {
        'console': {
            'class': 'logging.StreamHandler',
            'level': 'DEBUG',
            'formatter': 'colored',
            'stream': 'ext://sys.stdout',
        },
    },
    'loggers': {
        '': {
            'handlers': ['console'],
            'level': 'DEBUG',
        },
    },
}
