# first line: 338
def _fetch_tile(tile_url, wait, max_retries, headers: dict[str, str], timeout=None):
    array = _retryer(tile_url, wait, max_retries, headers, timeout=timeout)
    return array
