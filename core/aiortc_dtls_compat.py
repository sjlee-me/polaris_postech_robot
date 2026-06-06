"""
일부 단말 DTLS 인증서는 DER 버전 필드가 cryptography.load_der_x509_certificate에서
InvalidVersion으로 거절된다. aiortc는 지문 검증에 cryptography 객체가 필요해 여기서 깨진다.

이 모듈은 RTCDtlsTransport._validate_peer_identity를 감싸서,
cryptography 변환이 실패하면 PyOpenSSL X509.digest로 동일 SDP 지문과 비교한다.
"""
import logging

logger = logging.getLogger(__name__)

_PATCHED = False

_OPENSSL_DIGEST = {
    "sha-256": "sha256",
    "sha-384": "sha384",
    "sha-512": "sha512",
}


def _fingerprint_pyopenssl(cert, algorithm: str) -> str:
    name = _OPENSSL_DIGEST.get(algorithm.lower())
    if name is None:
        raise ValueError(algorithm)
    dig = cert.digest(name)
    if isinstance(dig, bytes):
        return dig.decode("ascii").upper()
    return str(dig).upper()


def _is_cryptography_invalid_version(err: BaseException) -> bool:
    if type(err).__name__ == "InvalidVersion":
        return True
    mod = getattr(type(err), "__module__", "") or ""
    return "cryptography" in mod and "InvalidVersion" in type(err).__name__


def apply_aiortc_dtls_peer_cert_patch() -> None:
    global _PATCHED
    if _PATCHED:
        return
    from aiortc import rtcdtlstransport as dtls

    X509_DIGEST_ALGORITHMS = dtls.X509_DIGEST_ALGORITHMS
    certificate_digest = dtls.certificate_digest
    State = dtls.State

    _orig = dtls.RTCDtlsTransport._validate_peer_identity

    def _validate_peer_identity(self, remoteParameters) -> None:
        cert_crypto = None
        try:
            cert_crypto = self._ssl.get_peer_certificate(as_cryptography=True)
        except Exception as e:
            if not _is_cryptography_invalid_version(e):
                raise
            logger.warning(
                "DTLS 상대 인증서를 cryptography로 파싱하지 못함 (%s). "
                "PyOpenSSL 지문으로만 검증합니다.",
                e,
            )
            cert_ssl = self._ssl.get_peer_certificate()
            if cert_ssl is None:
                self.__log_debug("x DTLS: peer certificate 없음")
                self._set_state(State.FAILED)
                return
            fingerprint_supported = 0
            fingerprint_valid = 0
            for f in remoteParameters.fingerprints:
                algorithm = f.algorithm.lower()
                if algorithm in X509_DIGEST_ALGORITHMS:
                    fingerprint_supported += 1
                    try:
                        fp = _fingerprint_pyopenssl(cert_ssl, algorithm)
                    except Exception as conv_err:
                        logger.debug("OpenSSL digest 실패: %s", conv_err)
                        fp = ""
                    if f.value.upper() == fp:
                        fingerprint_valid += 1
            if not fingerprint_supported or fingerprint_valid != fingerprint_supported:
                self.__log_debug("x DTLS handshake failed (fingerprint mismatch)")
                self._set_state(State.FAILED)
            return

        fingerprint_supported = 0
        fingerprint_valid = 0
        for f in remoteParameters.fingerprints:
            algorithm = f.algorithm.lower()
            if algorithm in X509_DIGEST_ALGORITHMS:
                fingerprint_supported += 1
                if f.value.upper() == certificate_digest(cert_crypto, algorithm):
                    fingerprint_valid += 1
        if not fingerprint_supported or fingerprint_valid != fingerprint_supported:
            self.__log_debug("x DTLS handshake failed (fingerprint mismatch)")
            self._set_state(State.FAILED)

    dtls.RTCDtlsTransport._validate_peer_identity = _validate_peer_identity
    _PATCHED = True
